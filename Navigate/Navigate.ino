/* Navigate - High-Level Arduino Due firmware (Sensor Hub slot)
 *
 * Stand-in for the Jetson Nano Navigation computer.
 * Consumes pose (GPS, compass, velocity) over Native USB from the simulator,
 * computes waypoint-following drive commands in a local Euclidean cm-frame,
 * and sends CAN 0x350 (NavDrive) + CAN 0x100 (NavStatus) to Drive-By-Wire.
 *
 * Per project convention (see Common.cpp in elcano/HighLevel and the wiki),
 * all in-system location math is in a flat Cartesian (east_cm, north_cm)
 * frame relative to a startup origin. Lat/lon is only used at the simulator
 * boundary; internal state and CAN messages are integer cm.
 *
 * CAN protocol per elcanoproject.org/wiki/Communication:
 *   0x350 NavDrive:    int16[0]=speed_cmPs, int16[1]=brake, int16[2]=angle_DegX10
 *   0x100 NavStatus:   byte0 bits  0x80=E-stop, 0x40=autonomous, 0x04=reverse
 *   0x101 GoalReached: byte0 0x80 set when final waypoint reached
 *   0x200 DBWStatus:   byte0 = DBW mode mirror (received, not yet acted on)
 *   0x400 Actual:      DBW actual speed/angle (received, not yet acted on)
 *   0x700-0x70A:       Log frames from DBW (received and written to SD card —
 *                      see "CAN-based logging" below)
 *
 * CAN-based logging:
 *   DBW emits log records over CAN as messages 0x700-0x70A, one set per
 *   loop tick (see wiki). This sketch listens for those frames, accumulates
 *   them between 0x70A "finalize" markers, and writes one CSV row per entry
 *   to an SD card on the Sensor Hub slot (CS pin D35, default no-jumpers
 *   config per the Bridge SD doc). Byte layouts of 0x701-0x709 are not yet
 *   specified by the wiki; the SD CSV captures raw hex bytes per CAN ID,
 *   losslessly. A future decoder (firmware update or Python post-processor)
 *   can split the hex into named subfields once the byte spec lands.
 *
 * Sim serial CSV format (line-based, easy to drive from a PC terminal):
 *   G,<lat_deg>,<lon_deg>     e.g. G,47.7600,-122.1917
 *   C,<heading_deg>           e.g. C,270.0
 *   V,<speed_cmPs>            e.g. V,120
 *   R                         mission reset (no payload)
 * Lines are terminated by '\n'.
 *
 * Self-test (enabled by default): SELF_TEST below injects a hardcoded pose
 * sequence that walks the trike through every waypoint, with no external
 * input needed. Useful for smoke-testing the sketch on a bare Due — flash
 * it, open Serial Monitor at 115200, and watch the IN/OUT pairs advance
 * through the loop. Comment out SELF_TEST to use normal USB serial pose
 * input instead. Disable before flashing to a real trike with live
 * throttle/steering actuators.
 */

#include <due_can.h>
#include <SPI.h>
#include <SD.h>
#include "Settings.h"
#include "NavMath.h"
#include "Waypoints.h"
#include "SimProtocol.h"

// SD card CS pin per Bridge SD doc, default Sensor-Hub-side config (no JP2
// jumpers). If JP2 jumpers route SPI to Router instead, SD on Sensor Hub is
// unavailable and SD.begin() will fail; the sketch then falls back to
// Serial-only logging output.
#define SD_CS_PIN 35

// Hardcoded pose-sequence self-test. Enabled by default — when on, the
// sketch ignores USB serial pose input and walks an internal timeline
// through every waypoint (see header comment). Comment out for normal
// pose-from-USB behavior.
//
// SAFETY: when enabled, the sketch emits 0x350 NavDrive frames commanding
// non-zero speed during the test. Safe on the Bridge bench (DBW in HIL
// with the simulator, or no actuators connected). Disable before flashing
// to a real trike with live throttle/steering actuators.
#define SELF_TEST

#ifdef SELF_TEST
// Set true by runSelfTest() on each step transition; consumed by
// computeAndSendDrive() to print exactly one input/output pair per step
// (instead of one per loop tick).
static bool selfTestPendingOutput = false;
#endif

#define NavStatus_CANID    0x100
#define GoalReached_CANID  0x101
#define DBWStatus_CANID    0x200
#define NavDrive_CANID     0x350
#define Actual_CANID       0x400

// Log message IDs (DBW emits these for CAN-based logging — see header comment)
#define LogHeader_CANID    0x700  // start-of-log marker
#define LogTime_CANID      0x701
#define LogRC_CANID        0x702
#define LogOp_CANID        0x703
#define LogAuto_CANID      0x704
#define LogDesired_CANID   0x705
#define LogThrottle_CANID  0x706
#define LogBrakes_CANID    0x707
#define LogSteer_CANID     0x708
#define LogPosition_CANID  0x709
#define LogFinalize_CANID  0x70A  // end-of-entry marker
#define LogID_FIRST        0x700
#define LogID_LAST         0x70A
#define LogID_COUNT        (LogID_LAST - LogID_FIRST + 1)  // 11

// Stale-buffer timeout: if we haven't seen a Log frame in this many ms,
// flush whatever's accumulated as a partial row. Protects against losing
// data if DBW dies mid-entry.
#define LOG_TIMEOUT_MS 1000

// ---- Origin and waypoints in cm-frame (filled at startup) ----
// Origin is set from waypoints[0] at startup; all internal pose math is
// done in (east_cm, north_cm) integer offsets from this origin.
static Origin nav_origin;
static int32_t waypoints_east_cm[NUM_WAYPOINTS];
static int32_t waypoints_north_cm[NUM_WAYPOINTS];

// ---- Navigation state (cm-frame, integer) ----
static int32_t currentEast_cm = 0;
static int32_t currentNorth_cm = 0;
static int32_t currentHeading_centiDeg = 0;   // 0 = north, 9000 = east
static int16_t currentSpeed_cmPs = 0;
static bool gpsValid = false;

static int waypointIdx = 0;
static bool missionComplete = false;

// ---- Sim serial line buffer ----
static const size_t LINE_BUF_SZ = 64;
static char lineBuf[LINE_BUF_SZ];
static size_t lineLen = 0;

// ---- CAN-based log capture (writes to SD card) ----
// Accumulates frames 0x701-0x709 between 0x70A "finalize" markers, writes
// one CSV row per entry. Each cell is the raw hex bytes of that CAN ID's
// frame, lossless. Empty cell = that ID didn't arrive in this entry.
struct LogEntryBuffer {
  bool received[LogID_COUNT];        // [0]=0x700, [1]=0x701, ..., [10]=0x70A
  uint8_t length[LogID_COUNT];       // bytes received per slot (0 if absent)
  uint8_t data[LogID_COUNT][8];      // raw bytes per slot
  uint32_t lastFrame_ms;             // millis() of most recent log frame
  bool hasContent;                   // any 0x701-0x709 received since last reset
};
static LogEntryBuffer logBuf;
static File logFile;
static bool sdAvailable = false;

// ---- CAN frames ----
static CAN_FRAME outgoing;

/* -------------------------------------------------------------------------- */
/* Serial parsing (CSV format)                                                */
/* -------------------------------------------------------------------------- */
static void handleLine(char* line) {
  if (line[0] == 0) return;
  // Single-char commands (no comma) handled first so they aren't rejected
  // by the tag,payload guard below.
  if (line[0] == 'R' && (line[1] == 0 || line[1] == ',')) {
    waypointIdx = 0;
    missionComplete = false;
    SerialUSB.println("RX R mission reset");
    return;
  }
  if (line[1] != ',') return;
  char tag = line[0];
  char* payload = line + 2;

  if (tag == 'G') {
    // G,<lat>,<lon> — convert to cm-frame at the boundary.
    char* comma = strchr(payload, ',');
    if (!comma) return;
    *comma = 0;
    double lat = atof(payload);
    double lon = atof(comma + 1);
    latlon_to_cm(nav_origin, lat, lon, currentEast_cm, currentNorth_cm);
    gpsValid = true;
    SerialUSB.print("RX G east_cm=");  SerialUSB.print(currentEast_cm);
    SerialUSB.print(" north_cm=");      SerialUSB.println(currentNorth_cm);
  } else if (tag == 'C') {
    // C,<heading_deg> — store as centidegrees.
    double hdg_deg = atof(payload);
    currentHeading_centiDeg = (int32_t)(hdg_deg * 100.0);
    SerialUSB.print("RX C heading_centiDeg=");
    SerialUSB.println(currentHeading_centiDeg);
  } else if (tag == 'V') {
    currentSpeed_cmPs = (int16_t)atoi(payload);
    SerialUSB.print("RX V speed_cmPs=");
    SerialUSB.println(currentSpeed_cmPs);
  } else {
    SerialUSB.print("RX unknown tag: ");
    SerialUSB.println(tag);
  }
}

static void readSimSerial() {
  while (SerialUSB.available() > 0) {
    char c = (char)SerialUSB.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < LINE_BUF_SZ - 1) {
      lineBuf[lineLen++] = c;
    } else {
      // overflow - drop the line
      lineLen = 0;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Self-test sequence (optional)                                              */
/*                                                                            */
/* When SELF_TEST is defined, this function injects a pose timeline that      */
/* walks the trike through every waypoint, with each step held for ~2s. No    */
/* external input is needed - just flash, open Serial Monitor, and watch the  */
/* waypoint number advance. The sequence assumes the default Waypoints.h      */
/* (4-corner ~22 m loop near UW Bothell).                                     */
/* -------------------------------------------------------------------------- */
#ifdef SELF_TEST
static void runSelfTest() {
  // Lat/lon stored here mirrors what the simulator/real GPS would emit;
  // they are converted to cm-frame at injection time (same path as the
  // 'G' serial command).
  static const struct {
    double lat, lon, hdg;
    int16_t speed;
    uint32_t hold_ms;
  } steps[] = {
    // Start at WP0 facing north, stopped. Sketch will advance to wp=1.
    { 47.7600,  -122.1917,    0.0,   0, 2000 },
    // Roll north toward WP1.
    { 47.7601,  -122.1917,    0.0, 100, 2000 },
    // Reach WP1. Sketch advances to wp=2.
    { 47.7602,  -122.1917,    0.0, 100, 2000 },
    // Turn east, roll toward WP2.
    { 47.7602,  -122.19155,  90.0, 100, 2000 },
    // Reach WP2. Sketch advances to wp=3.
    { 47.7602,  -122.1914,   90.0, 100, 2000 },
    // Turn south, roll toward WP3.
    { 47.76015, -122.1914,  180.0, 100, 2000 },
    // Reach WP3 (still moving). Sketch advances past final waypoint and
    // sets missionComplete. Expected output: Destination arrived, stopping.
    { 47.7600,  -122.1914,  180.0, 100, 2000 },
    // Trike has now physically stopped (speed=0 reported back). Verifies
    // mission-complete state stays sticky after the trike obeys the brake
    // command. Expected output: Stopped.
    { 47.7600,  -122.1914,  180.0,   0, 2000 },
  };
  static const int N = sizeof(steps) / sizeof(steps[0]);
  static int idx = 0;
  static uint32_t next_ms = 0;

  if (idx >= N) return;  // sequence complete
  if (millis() < next_ms) return;

  // Inject pose by routing through the same conversion the 'G' tag would.
  latlon_to_cm(nav_origin, steps[idx].lat, steps[idx].lon,
               currentEast_cm, currentNorth_cm);
  currentHeading_centiDeg = (int32_t)(steps[idx].hdg * 100.0);
  currentSpeed_cmPs = steps[idx].speed;
  gpsValid = true;
  next_ms = millis() + steps[idx].hold_ms;

  SerialUSB.println();
  SerialUSB.print("--- SELF_TEST step ");
  SerialUSB.print(idx + 1);
  SerialUSB.print("/");
  SerialUSB.print(N);
  SerialUSB.println(" ---");
  SerialUSB.print("  IN  : east_cm=");  SerialUSB.print(currentEast_cm);
  SerialUSB.print(" north_cm=");        SerialUSB.print(currentNorth_cm);
  SerialUSB.print(" heading=");         SerialUSB.print(steps[idx].hdg, 1);
  SerialUSB.print(" speed=");           SerialUSB.println(steps[idx].speed);

  idx++;
  selfTestPendingOutput = true;
}
#endif

/* -------------------------------------------------------------------------- */
/* Waypoint progression                                                       */
/* -------------------------------------------------------------------------- */
static void updateWaypoint() {
  if (!gpsValid || missionComplete) return;
  int32_t dist = distance_cm(currentEast_cm, currentNorth_cm,
                             waypoints_east_cm[waypointIdx],
                             waypoints_north_cm[waypointIdx]);
  if (dist < WAYPOINT_RADIUS_CM) {
    waypointIdx++;
    if (waypointIdx >= NUM_WAYPOINTS) {
      missionComplete = true;
      // 0x101 GoalReached with mission-complete bit
      outgoing.id = GoalReached_CANID;
      outgoing.length = 1;
      outgoing.data.uint8[0] = 0x80;
      Can0.sendFrame(outgoing);
    } else {
      // Advance notification: byte0 = next waypoint index (low bits)
      outgoing.id = GoalReached_CANID;
      outgoing.length = 1;
      outgoing.data.uint8[0] = (uint8_t)(waypointIdx & 0x7F);
      Can0.sendFrame(outgoing);
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Drive command                                                              */
/* -------------------------------------------------------------------------- */
static void computeAndSendDrive() {
  int16_t speedCmd = 0;
  int16_t brakeCmd = 100;  // default: brake on
  int16_t angleCmd = 0;
  int32_t dist_cm = 0;
  int32_t bearing_cD = 0;
  int32_t err_cD = 0;

  if (gpsValid && !missionComplete) {
    int32_t tgt_e = waypoints_east_cm[waypointIdx];
    int32_t tgt_n = waypoints_north_cm[waypointIdx];
    dist_cm    = distance_cm(currentEast_cm, currentNorth_cm, tgt_e, tgt_n);
    bearing_cD = bearing_centiDeg(currentEast_cm, currentNorth_cm, tgt_e, tgt_n);
    err_cD     = wrap_centiDeg_signed(bearing_cD - currentHeading_centiDeg);
    // steer_DegX10 = Kp * err_deg = (Kp * err_centiDeg) / 100
    int32_t steer = ((int32_t)Kp_STEERING * err_cD) / 100;
    int32_t cap = (int32_t)MAX_STEER_DEG * 10;
    if (steer >  cap) steer =  cap;
    if (steer < -cap) steer = -cap;
    angleCmd = (int16_t)steer;
    speedCmd = CRUISE_CMPS;
    brakeCmd = 0;
  }

  // Decide whether to print this tick. In SELF_TEST mode print exactly once
  // per step (paired with the IN line). In normal mode print every loop tick.
#ifdef SELF_TEST
  bool shouldPrint = selfTestPendingOutput;
  if (shouldPrint) selfTestPendingOutput = false;
#else
  bool shouldPrint = (gpsValid && !missionComplete);
#endif

  if (shouldPrint) {
    if (gpsValid && !missionComplete) {
      SerialUSB.print("  OUT : waypoint="); SerialUSB.print(waypointIdx);
      SerialUSB.print(" dist_cm=");         SerialUSB.print(dist_cm);
      SerialUSB.print(" bearing=");         SerialUSB.print(bearing_cD / 100.0, 1);
      SerialUSB.print(" heading=");         SerialUSB.print(currentHeading_centiDeg / 100.0, 1);
      SerialUSB.print(" err=");             SerialUSB.print(err_cD / 100.0, 1);
      SerialUSB.print(" -> speed=");        SerialUSB.print(speedCmd);
      SerialUSB.print(" brake=");           SerialUSB.print(brakeCmd);
      SerialUSB.print(" angle=");           SerialUSB.println(angleCmd);
    } else if (missionComplete) {
      if (currentSpeed_cmPs > 0) {
        SerialUSB.println("  OUT : Destination arrived, stopping -> speed=0 brake=100 angle=0");
      } else {
        SerialUSB.println("  OUT : Stopped -> speed=0 brake=100 angle=0");
      }
    } else {
      SerialUSB.println("  OUT : (no GPS yet) -> speed=0 brake=100 angle=0");
    }
  }

  // 0x350 NavDrive
  outgoing.id = NavDrive_CANID;
  outgoing.length = 6;
  outgoing.data.int16[0] = speedCmd;
  outgoing.data.int16[1] = brakeCmd;
  outgoing.data.int16[2] = angleCmd;
  Can0.sendFrame(outgoing);

  // 0x100 NavStatus - byte0 bits: 0x40 autonomous, 0x80 e-stop, 0x04 reverse
  outgoing.id = NavStatus_CANID;
  outgoing.length = 1;
  uint8_t status = 0x40;  // autonomous intent
  if (missionComplete) status |= 0x80;  // mission-complete -> request stop
  outgoing.data.uint8[0] = status;
  Can0.sendFrame(outgoing);
}

/* -------------------------------------------------------------------------- */
/* CAN-based log capture (writes to SD card)                                  */
/* -------------------------------------------------------------------------- */

// Reset the in-memory log buffer for the next entry.
static void resetLogBuffer() {
  for (int i = 0; i < LogID_COUNT; i++) {
    logBuf.received[i] = false;
    logBuf.length[i] = 0;
  }
  logBuf.hasContent = false;
  logBuf.lastFrame_ms = 0;
}

// Open SD card and start a new log file (LOG00.CSV, LOG01.CSV, ...).
// Falls back to no-op (sdAvailable = false) if SD isn't present; sketch
// continues to run normally without writing logs.
static void initSdLog() {
  resetLogBuffer();
  SerialUSB.print("Initializing SD card on D");
  SerialUSB.print(SD_CS_PIN);
  SerialUSB.print("... ");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(100);
  if (!SD.begin(SD_CS_PIN)) {
    SerialUSB.println("not found. CAN log -> SD disabled.");
    SerialUSB.println("(If SD is on Router via JP2 jumpers, that's expected.)");
    return;
  }
  SerialUSB.println("OK.");
  // Pick the first unused LOG##.CSV
  char filename[13];
  for (int i = 0; i < 100; i++) {
    sprintf(filename, "LOG%02d.CSV", i);
    if (!SD.exists(filename)) break;
  }
  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    SerialUSB.print("Could not open ");
    SerialUSB.println(filename);
    return;
  }
  SerialUSB.print("CAN log -> SD: ");
  SerialUSB.println(filename);
  // CSV header. Each cell holds raw hex bytes of that CAN ID's frame
  // (lossless capture; byte layouts of 0x701-0x709 not yet specified by
  // the wiki — decoder is a future enhancement).
  logFile.println("rx_ms,time_701,rc_702,op_703,nav_cmd_704,active_cmd_705,throttle_706,brakes_707,steer_708,position_709,util_70A");
  logFile.flush();
  sdAvailable = true;
}

// Write the accumulated log buffer as one CSV row, then reset.
// Called when 0x70A is received OR when the stale-buffer timeout fires.
static uint32_t logRowsWritten = 0;
static void writeLogRow() {
  if (!logBuf.hasContent) {
    resetLogBuffer();
    return;
  }
  if (!sdAvailable) {
    resetLogBuffer();
    return;
  }
  logFile.print(millis()); logFile.print(",");
  // Write slots 1..10 (0x701..0x70A) — slot 0 (0x700) is the session header,
  // not a per-row field.
  for (int slot = 1; slot < LogID_COUNT; slot++) {
    if (logBuf.received[slot]) {
      for (int b = 0; b < logBuf.length[slot]; b++) {
        if (logBuf.data[slot][b] < 0x10) logFile.print("0");
        logFile.print(logBuf.data[slot][b], HEX);
      }
    }
    if (slot < LogID_COUNT - 1) logFile.print(",");
  }
  logFile.println();
  logFile.flush();
  logRowsWritten++;
  SerialUSB.print("Log row written (total: ");
  SerialUSB.print(logRowsWritten);
  SerialUSB.println(")");
  resetLogBuffer();
}

// Handle one received CAN frame whose ID is in 0x700-0x70A.
static void onLogFrame(const CAN_FRAME& f) {
  int slot = (int)f.id - LogID_FIRST;
  if (slot < 0 || slot >= LogID_COUNT) return;

  if (f.id == LogHeader_CANID) {
    // 0x700 marks a new logging session from DBW. Flush any partial
    // buffer (in case DBW restarted mid-entry) and reset for a fresh
    // sequence of 0x701-0x70A frames.
    if (logBuf.hasContent) writeLogRow();
    SerialUSB.println("RX 0x700 LogHeader (new session)");
    resetLogBuffer();
    return;
  }

  // 0x701..0x70A: stash the frame in its slot.
  uint8_t len = f.length;
  if (len > 8) len = 8;
  logBuf.received[slot] = true;
  logBuf.length[slot] = len;
  for (int b = 0; b < len; b++) logBuf.data[slot][b] = f.data.uint8[b];
  logBuf.lastFrame_ms = millis();
  if (f.id != LogFinalize_CANID) logBuf.hasContent = true;

  if (f.id == LogFinalize_CANID) {
    writeLogRow();
  }
}

// If a partial buffer has been sitting unfinished for too long (DBW lost
// the bus, mis-sequenced its frames, etc.), flush it as a partial row.
static void checkLogTimeout() {
  if (!logBuf.hasContent) return;
  if (millis() - logBuf.lastFrame_ms < LOG_TIMEOUT_MS) return;
  SerialUSB.println("Log buffer timeout - flushing partial entry");
  writeLogRow();
}

/* -------------------------------------------------------------------------- */
/* Arduino entry points                                                       */
/* -------------------------------------------------------------------------- */
void setup() {
  SerialUSB.begin(115200);
  // Don't block on SerialUSB - the vehicle may run without a PC attached.

  // Visible startup countdown so the Serial Monitor has time to attach
  // after upload/reset. The Native USB port re-enumerates after upload,
  // which can hide the first few prints otherwise.
  for (int i = 5; i > 0; i--) {
    SerialUSB.print("Starting in ");
    SerialUSB.print(i);
    SerialUSB.println("...");
    delay(1000);
  }

  if (Can0.begin(CAN_BPS_500K)) {
    SerialUSB.println("CAN init success");
  } else {
    SerialUSB.println("CAN init failed");
  }
  // Listen for DBW status and actuals so we can react to them later.
  Can0.watchFor(DBWStatus_CANID);
  Can0.watchFor(Actual_CANID);

  // Listen for log frames from DBW (0x700-0x70A) for CAN-based SD logging.
  // Use watchForRange so the entire log ID range fits in one CAN mailbox —
  // the SAM3X has only 8 mailboxes per port, and per-ID watchFor would burn
  // 11 of them (silently failing past mailbox 8).
  Can0.watchForRange(LogID_FIRST, LogID_LAST);

  // Initialize the local Euclidean frame using waypoint 0 as the origin,
  // then convert all waypoints to cm-frame for fast integer math at
  // runtime.
  origin_init(nav_origin, waypoints[0].lat_deg, waypoints[0].lon_deg);
  for (int i = 0; i < NUM_WAYPOINTS; i++) {
    latlon_to_cm(nav_origin, waypoints[i].lat_deg, waypoints[i].lon_deg,
                 waypoints_east_cm[i], waypoints_north_cm[i]);
  }

  SerialUSB.print("Waypoints: ");
  SerialUSB.println(NUM_WAYPOINTS);
  SerialUSB.print("Origin: lat=");  SerialUSB.print(nav_origin.lat, 6);
  SerialUSB.print(" lon=");          SerialUSB.println(nav_origin.lon, 6);

  // Start the SD logger (no-op if SD card isn't present).
  initSdLog();

#ifdef SELF_TEST
  SerialUSB.println();
  SerialUSB.println("=== SELF_TEST mode ===");
  SerialUSB.print("This test walks the trike through ");
  SerialUSB.print(NUM_WAYPOINTS);
  SerialUSB.println(" hardcoded waypoints (a ~22m square loop)");
  SerialUSB.println("without external input. Each step shows:");
  SerialUSB.println("  IN  = simulated pose injected (east_cm/north_cm/heading/speed)");
  SerialUSB.println("  OUT = nav decision (target waypoint, dist_cm/bearing, drive command)");
  SerialUSB.println();
#endif
}

void loop() {
  const uint32_t loopPeriod_ms = 1000UL / NAV_LOOP_HZ;
  uint32_t start_ms = millis();

#ifdef SELF_TEST
  runSelfTest();
#else
  readSimSerial();
#endif
  updateWaypoint();
  computeAndSendDrive();

  // Drain any incoming CAN frames so the buffer doesn't fill up.
  CAN_FRAME incoming;
  while (Can0.available() > 0) {
    Can0.read(incoming);
    if (incoming.id >= LogID_FIRST && incoming.id <= LogID_LAST) {
      onLogFrame(incoming);
    }
    // Future: also react to DBWStatus (0x200) / Actual (0x400).
  }

  // Flush a partial log entry if it's been sitting too long.
  checkLogTimeout();

  uint32_t elapsed = millis() - start_ms;
  if (elapsed < loopPeriod_ms) delay(loopPeriod_ms - elapsed);
}
