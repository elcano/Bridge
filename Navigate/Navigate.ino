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
#include "Settings.h"
#include "NavMath.h"
#include "Waypoints.h"
#include "SimProtocol.h"

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
/* Arduino entry points                                                       */
/* -------------------------------------------------------------------------- */
void setup() {
  SerialUSB.begin(115200);
  // Don't block on SerialUSB - the vehicle may run without a PC attached.
  if (Can0.begin(CAN_BPS_500K)) {
    SerialUSB.println("CAN init success");
  } else {
    SerialUSB.println("CAN init failed");
  }
  // Listen for DBW status and actuals so we can react to them later.
  Can0.watchFor(DBWStatus_CANID);
  Can0.watchFor(Actual_CANID);

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
  // Visible countdown so step 1 isn't lost in the Native USB re-enumerate
  // window after upload/reset. Even if the Serial Monitor opens partway
  // through, the user catches enough of the countdown to know what's
  // happening before step 1 fires.
  for (int i = 5; i > 0; i--) {
    SerialUSB.print("Starting sequence in ");
    SerialUSB.print(i);
    SerialUSB.println("...");
    delay(1000);
  }
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
    // Placeholder - future: react to DBW DBWStatus / Actual.
  }

  uint32_t elapsed = millis() - start_ms;
  if (elapsed < loopPeriod_ms) delay(loopPeriod_ms - elapsed);
}
