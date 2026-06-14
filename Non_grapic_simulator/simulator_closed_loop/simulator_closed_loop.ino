/*
 * Simulator — Closed-Loop Integration variant
 * Router Arduino Due
 *
 * SISTER SKETCH to ../simulator_stage1/simulator_stage1.ino.
 *   simulator_stage1     : standalone / standalone-Due use, no CAN, simple
 *                          test mode with fixed throttle. Owned by Minhee.
 *   simulator_closed_loop: end-to-end closed-loop test on the Bridge board.
 *                          Reads real DBW inputs (throttle / L_TURN / R_TURN),
 *                          publishes pose + actual wheel angle on CAN so the
 *                          Sensor Hub Nav + DBW close their loops through it.
 *
 * The two sketches share the same integer physics engine (sin1000, cos1000,
 * computeSpeed, updateAngle, updatePosition, sendWheelPulse,
 * sendAngleSensor). Those functions are mirrored here verbatim. If the
 * physics changes in simulator_stage1.ino, mirror it here too (or factor
 * out to a shared header later).
 *
 * --- Hardware I/O ---
 * Inputs from DBW Arduino (via Bridge harness):
 *   A0  -> THROTTLE      (from DBW DAC0)
 *   D42 -> BRAKE_VOLT    (from DBW D40)
 *   D48 -> BRAKE_ON      (from DBW D44)
 *   D4  -> L_TURN        (from DBW D26): HIGH while DBW wants to turn left
 *   D2  -> R_TURN        (from DBW D28): HIGH while DBW wants to turn right
 *
 * Outputs to DBW Arduino:
 *   D47  -> IRPT_WHEEL   (wheel-tick pulse, rate scales with speed)
 *   DAC0 -> L_SENSE      (left wheel angle pot voltage)
 *   DAC1 -> R_SENSE      (right wheel angle pot voltage)
 *
 * CAN frames Router transmits (Router does not RX any frames):
 *   0x4C0  position    (east_cm int32 + north_cm int32, 8 bytes)
 *   0x4E0  heading     (heading_centiDeg int16, 2 bytes)
 *   0x4F0  speed       (speed_cmPs int16, 2 bytes)
 *   0x430  steer-actual (angle_DegX10 int16, 2 bytes) — substitutes for the
 *                       L_SENSE/R_SENSE analog feedback that the bridge
 *                       harness can't carry reliably. DBW uses this as the
 *                       measured wheel angle for its PID. On the real trike
 *                       DBW would read its own analog sensors instead.
 *
 * --- Stage 2 ASCII command pipe (PC -> Router via USB) ---
 * Format: "CANID,nbytes,data1,data2,...\n"
 * Example: "350,6,1500,0,1,-21" -> CAN ID 0x350, speed=1500 cm/s, brake=0,
 *          mode=1, angle=-21 (=-2.1 deg left). Lets a PC script drive Router
 *          without DBW being present.
 *
 * Logs CSV to SD card if available, falls back to USB Serial (LOG_PORT).
 * CSV: time_ms, X_mm, Y_mm, heading_tenths, speed_mmPs, angle_tenths, throttle, brakeOn
 *
 * All arithmetic uses integers per professor's requirement.
 * Angles stored as tenths of a degree. Positions in mm. Speed in mm/s.
 *
 * Reference: https://www.elcanoproject.org/wiki/Communication
 */

// ============================================================================
// SETUP REQUIRED — this sketch will not compile until the Arduino IDE's
// Sketchbook Location is pointed at the parent folder of this sketch:
//   File -> Preferences -> Sketchbook location: <repo>/Non_grapic_simulator
// The IDE then finds the shared `simulator_physics` library at
//   ../libraries/simulator_physics/
// See ../README.md for full setup details + troubleshooting.
// ============================================================================

#include <SPI.h>
#include <SD.h>
#include <due_can.h>

// ===== Logging / USB Serial port =====
// LOG_PORT is used for ALL host-side I/O: CSV log, debug prints, Stage 2
// ASCII command input, Stage 3 binary 0x251/0x4C0 packets.
//
//   USE_NATIVE_USB defined (default for this sketch) -> SerialUSB (Native USB port)
//   USE_NATIVE_USB commented out                     -> Serial    (Programming port)
//
// This sketch defaults to SerialUSB because it's intended to be flashed onto
// a Router Due sitting on the Bridge board. The Bridge harness wires
// Router's D0/D1 (the Serial UART pins) to DBW's Serial1, so Serial.print()
// would also send bytes onto that inter-board wire and garbage DBW's debug
// stream. SerialUSB routes through the Native USB jack and leaves D0/D1
// quiet.
//
// If you ever want to run this sketch on a standalone Due (no Bridge), comment
// out the line below and plug into the Programming USB port instead.
#define USE_NATIVE_USB

#ifdef USE_NATIVE_USB
  #define LOG_PORT SerialUSB
#else
  #define LOG_PORT Serial
#endif

// ===== Pin Definitions =====
// From DBW DAC0
#define THROTTLE_PIN    A0
// From DBW D40
#define BRAKE_VOLT_PIN  42
// From DBW D44
#define BRAKE_ON_PIN    48
// From DBW D26 — HIGH while DBW wants to turn left
#define L_TURN_PIN      4
// From DBW D28 — HIGH while DBW wants to turn right
#define R_TURN_PIN      2
// To DBW D47
#define IRPT_WHEEL_PIN  47
// To DBW A10
#define L_SENSE_PIN     DAC0
// To DBW A11
#define R_SENSE_PIN     DAC1

// ===== SD Card Pin =====
#define SD_CS_PIN       37

// ===== CAN ID Definitions =====
// Per https://www.elcanoproject.org/wiki/Communication
// Nav->DBW: speed(cm/s), brake, mode, angle(deg x10)
#define CAN_DRIVE         0x350
// Router->Sensor Hub: vehicle position east_cm, north_cm
#define CAN_POSITION      0x4C0
// Router->Sensor Hub: heading_centiDeg (compass only)
#define CAN_HEADING       0x4E0
// Router->Sensor Hub: speed_cmPs
#define CAN_SPEED         0x4F0
// Router->DBW: simulated actual wheel angle (deg x10).
// Substitutes for L_SENSE/R_SENSE analog feedback that is unavailable on
// this bridge. On the real trike, DBW would read its analog sensors.
#define CAN_STEER_ACTUAL  0x430
// Nav->all: GPS origin lat/lon
#define CAN_ORIGIN        0x251

// ===== Origin GPS coordinates (UW Bothell) =====
// latitude degrees
#define ORIGIN_LAT       47
// latitude fraction (0-9,999,999)
#define ORIGIN_LAT_FRAC  760934
// longitude degrees
#define ORIGIN_LON       122
// longitude fraction (0-999,999), West
#define ORIGIN_LON_FRAC  189963

// ===== Speed Model Constants =====
#define FRICTION_NUM            9296
#define FRICTION_DEN            10000
#define MIN_EFFECTIVE_THROTTLE  65
#define MAX_EFFECTIVE_THROTTLE  227
// 2 m/s cap — slow enough that turn radius (~3 m) fits within the waypoint
// capture radius. simulator_stage1 uses 13600 (13.6 m/s); that's too fast
// for the closed-loop Nav waypoint logic to track without overshooting.
#define MAX_SPEED_mmPs          2000
#define THROTTLE_HISTORY        10
#define THROTTLE_DELAY_START    3
#define THROTTLE_DELAY_END      10

// ===== Vehicle Settings =====
#define WHEEL_DIAMETER_MM    495
#define WHEEL_CIRCUM_MM      1555
#define LOOP_TIME_MS         100
#define MAX_ANGLE_TENTHS     250
// ===== Wheel Angle Sensor =====
#define ANGLE_CHANGE_TENTHS  20
#define L_STRAIGHT  722
#define L_MIN       779
#define L_MAX       639
#define R_STRAIGHT  731
#define R_MIN       673
#define R_MAX       786
// ===== Global Variables (all integer) =====
int throttleHistory[THROTTLE_HISTORY];
int historyIndex = 0;

int speed_mmPs     = 0;
int prevSpeed_mmPs = 0;
int heading_tenths = 0;
int angle_tenths   = 0;

long X_mm = 0;
long Y_mm = 0;

unsigned long nextPulseTime_ms = 0;

// ===== Stage 2: ASCII command state =====
bool useScriptCommand  = false;
// commanded speed from PC (cm/s)
int  cmd_speed_cmPs    = 0;
// 0=off, 1=hold(12V), 2=on(24V)
int  cmd_brake         = 0;
// 0=init,1=RC,2=operator,3=auto-RC,4=auto-op
int  cmd_mode          = 0;
// commanded steer angle (degrees x10)
int  cmd_angle_tenths  = 0;

// ===== SD Card Variables =====
File logFile;
bool sdAvailable = false;

// ===== Function Declarations =====
// sin1000, cos1000, computeSpeed, updateAngle, updatePosition,
// sendWheelPulse, sendAngleSensor are provided by ../simulator_physics.h.
void readScriptCommand();
void sendPosition();
void sendOrigin();
bool sendPositionCAN();
bool sendHeadingCAN();
bool sendSpeedCAN();
bool sendSteerActualCAN();

// Shared physics engine — provided by the in-repo Arduino library at
// Non_grapic_simulator/libraries/simulator_physics. Provides: sin1000,
// cos1000, computeSpeed, updateAngle, updatePosition, sendWheelPulse,
// sendAngleSensor. See the header for macro/global preconditions.
// Included AFTER the global variable declarations above because the inline
// function bodies reference those globals (prevSpeed_mmPs, angle_tenths,
// X_mm, Y_mm, historyIndex, throttleHistory[], nextPulseTime_ms) by name.
// SETUP: in Arduino IDE, set File -> Preferences -> Sketchbook location to
// <repo>/Non_grapic_simulator so the IDE finds the library.
#include <simulator_physics.h>

// ===========================================================================
// Setup
// ===========================================================================
void setup() {
  LOG_PORT.begin(115200);
#ifdef USE_NATIVE_USB
  // Native USB enumerates after sketch boot. Bounded wait so the sketch
  // boots even when no host is attached.
  uint32_t waitStart = millis();
  while (!LOG_PORT && (millis() - waitStart) < 3000);
#endif

  // Configure pins
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(BRAKE_VOLT_PIN, INPUT);
  pinMode(BRAKE_ON_PIN, INPUT);
  pinMode(L_TURN_PIN, INPUT);
  pinMode(R_TURN_PIN, INPUT);
  // Explicitly disable the SAM3X internal pullups on the L_TURN/R_TURN
  // inputs. Otherwise the boot pullup state is non-deterministic and
  // digitalRead can return HIGH even when DBW is actively pulling LOW.
  g_APinDescription[L_TURN_PIN].pPort->PIO_PUDR = g_APinDescription[L_TURN_PIN].ulPin;
  g_APinDescription[R_TURN_PIN].pPort->PIO_PUDR = g_APinDescription[R_TURN_PIN].ulPin;
  pinMode(IRPT_WHEEL_PIN, OUTPUT);
  digitalWrite(IRPT_WHEEL_PIN, LOW);

  // Due DAC is 12-bit (0-4095). Default analogWrite resolution is 8-bit,
  // which truncates the lSense*4 / rSense*4 values in sendAngleSensor and
  // saturates DBW's A10/A11 readings. Set 12-bit so calibration matches.
  analogWriteResolution(12);

  for (int i = 0; i < THROTTLE_HISTORY; i++) throttleHistory[i] = 0;

  // Initialize CAN at 500 kbps (matches DBW + Sensor Hub).
  // Router only transmits — no RX mailboxes needed.
  if (Can0.begin(CAN_BPS_500K)) {
    LOG_PORT.println("CAN init OK");
  } else {
    LOG_PORT.println("CAN init FAILED");
  }

  // Stage 3 binary USB origin (0x251) intentionally not sent at boot —
  // Sensor Hub Nav reads pose over CAN (0x4C0/0x4E0/0x4F0). The
  // sendOrigin() function is kept defined for backwards compat with the
  // older PC simulator pipe.

  // Initialize SD card
  LOG_PORT.print("Initializing SD card on pin D");
  LOG_PORT.print(SD_CS_PIN);
  LOG_PORT.print("... ");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(100);
  if (SD.begin(SD_CS_PIN)) {
    LOG_PORT.println("SD card found!");
    char filename[13];
    for (int i = 0; i < 100; i++) {
      sprintf(filename, "SIM%02d.CSV", i);
      if (!SD.exists(filename)) break;
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
      sdAvailable = true;
      LOG_PORT.print("Logging to SD: ");
      LOG_PORT.println(filename);
    } else {
      LOG_PORT.println("Failed to open file. Using USB Serial.");
    }
  } else {
    LOG_PORT.println("SD not found. Using USB Serial.");
  }

  if (sdAvailable) {
    logFile.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,throttle,brakeOn");
    logFile.flush();
  } else {
    LOG_PORT.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,throttle,brakeOn");
  }
  LOG_PORT.println("Simulator closed-loop variant started.");
}

// ===========================================================================
// Main Loop
// ===========================================================================
void loop() {
  // Runs continuously — no auto-halt; Sensor Hub keeps receiving frames for
  // the whole drive.
  uint32_t startTime = millis();

  // Stage 2: read ASCII command from PC if available
  readScriptCommand();

  int throttle;
  bool brakeOn;
  bool lTurn;
  bool rTurn;

  if (useScriptCommand) {
    // Drive command received from PC simulator (Stage 2).
    // speed and angle are set directly; skip throttle model.
    // cm/s -> mm/s
    speed_mmPs   = cmd_speed_cmPs * 10;
    angle_tenths = cmd_angle_tenths;
    brakeOn      = (cmd_brake > 0);
    throttle     = 0;
    lTurn        = false;
    rTurn        = false;
  } else {
    // Real inputs from DBW (closes the loop: Nav -> CAN -> DBW -> pins -> here).
    //   THROTTLE_PIN (A0)  <- DBW DAC0 throttle output, 0-1023 analog
    //   BRAKE_ON_PIN (D48) <- DBW BRAKE_ON_PIN  (D44)
    //   L_TURN_PIN   (D4)  <- DBW LEFT_TURN_PIN  (D26): HIGH = turn left
    //   R_TURN_PIN   (D2)  <- DBW RIGHT_TURN_PIN (D28): HIGH = turn right
    int rawThrottle = analogRead(THROTTLE_PIN);
    throttle = rawThrottle / 4;
    // BRAKE derived from throttle below the effective threshold. When Nav
    // commands speed=0, DBW's throttle PID outputs 0 and we treat that as brake.
    brakeOn  = (throttle < MIN_EFFECTIVE_THROTTLE);

    // Steering via two digital wires from DBW (L_TURN D4, R_TURN D2).
    lTurn = (digitalRead(L_TURN_PIN) == HIGH);
    rTurn = (digitalRead(R_TURN_PIN) == HIGH);
    angle_tenths = updateAngle(lTurn, rTurn);

    static int dbgCount = 0;
    if (++dbgCount >= 10) {
      dbgCount = 0;
      LOG_PORT.print("# L=");         LOG_PORT.print(lTurn ? 1 : 0);
      LOG_PORT.print(" R=");          LOG_PORT.print(rTurn ? 1 : 0);
      LOG_PORT.print(" ang_tenths="); LOG_PORT.print(angle_tenths);
      LOG_PORT.print(" speed=");      LOG_PORT.println(speed_mmPs);
    }

    throttleHistory[historyIndex] = throttle;
    historyIndex = (historyIndex + 1) % THROTTLE_HISTORY;
    speed_mmPs   = computeSpeed(throttle, brakeOn);
  }

  // Update vehicle position
  updatePosition(speed_mmPs, heading_tenths, angle_tenths);

  // Simulated sensor outputs to DBW (wheel-tick + L_SENSE/R_SENSE DACs)
  sendWheelPulse(speed_mmPs);
  sendAngleSensor(angle_tenths);

  // Broadcast position, heading, speed, steer-actual on CAN — one frame each.
  static uint32_t txOk = 0, txFail = 0;
  if (sendPositionCAN())    txOk++; else txFail++;
  if (sendHeadingCAN())     txOk++; else txFail++;
  if (sendSpeedCAN())       txOk++; else txFail++;
  if (sendSteerActualCAN()) txOk++; else txFail++;
  static uint32_t lastTxDbg_ms = 0;
  if (millis() - lastTxDbg_ms > 2000) {
    lastTxDbg_ms = millis();
    LOG_PORT.print("# Router TX txOk="); LOG_PORT.print(txOk);
    LOG_PORT.print(" txFail=");          LOG_PORT.println(txFail);
  }

  // CSV log row
  if (sdAvailable) {
    logFile.print(millis());       logFile.print(",");
    logFile.print(X_mm);           logFile.print(",");
    logFile.print(Y_mm);           logFile.print(",");
    logFile.print(heading_tenths); logFile.print(",");
    logFile.print(speed_mmPs);     logFile.print(",");
    logFile.print(angle_tenths);   logFile.print(",");
    logFile.print(throttle);       logFile.print(",");
    logFile.println(brakeOn ? 1 : 0);
    logFile.flush();
  } else {
    LOG_PORT.print(millis());       LOG_PORT.print(",");
    LOG_PORT.print(X_mm);           LOG_PORT.print(",");
    LOG_PORT.print(Y_mm);           LOG_PORT.print(",");
    LOG_PORT.print(heading_tenths); LOG_PORT.print(",");
    LOG_PORT.print(speed_mmPs);     LOG_PORT.print(",");
    LOG_PORT.print(angle_tenths);   LOG_PORT.print(",");
    LOG_PORT.print(throttle);       LOG_PORT.print(",");
    LOG_PORT.println(brakeOn ? 1 : 0);
  }

  uint32_t elapsed = millis() - startTime;
  if (elapsed < LOOP_TIME_MS) delay(LOOP_TIME_MS - elapsed);
}

// ===========================================================================
// Stage 2: ASCII drive command from PC via USB Serial
// ===========================================================================
// Format: "CANID,nbytes,data1,data2,...\n"
// Example: "350,6,1500,0,1,-21"
//   CAN ID 0x350, 6 data bytes
//   speed=1500 cm/s, brake=0, mode=1, angle=-21 (=-2.1 deg left)
// Optional execution time prefix (ms): "1000,350,6,1500,0,1,-21"
// Lines starting with '#' are comments and ignored.
void readScriptCommand() {
  static char buf[64];
  static int  bufIdx = 0;

  while (LOG_PORT.available()) {
    char c = (char)LOG_PORT.read();
    if (c == '\n' || c == '\r') {
      buf[bufIdx] = '\0';
      bufIdx = 0;

      // Skip empty lines and comments
      if (buf[0] == '\0' || buf[0] == '#') return;

      // Parse comma-separated integer fields
      int fields[12];
      int nFields = 0;
      char *ptr = buf;
      while (*ptr && nFields < 12) {
        fields[nFields++] = (int)strtol(ptr, &ptr, 0);
        if (*ptr == ',') ptr++;
      }
      if (nFields < 3) return;

      // If first value > 0x7FF it is an execution time prefix; skip it
      int idx = (fields[0] > 0x7FF) ? 1 : 0;
      if (nFields < idx + 3) return;

      int canId = fields[idx];
      // fields[idx+1] = nbytes (not needed directly)
      // speed (cm/s)
      int d0 = (nFields > idx+2) ? fields[idx+2] : 0;
      // brake
      int d1 = (nFields > idx+3) ? fields[idx+3] : 0;
      // mode
      int d2 = (nFields > idx+4) ? fields[idx+4] : 0;
      // angle (deg x10)
      int d3 = (nFields > idx+5) ? fields[idx+5] : 0;

      if (canId == CAN_DRIVE) {
        // 0x350: speed(cm/s), brake, mode, steer angle(deg x10)
        cmd_speed_cmPs   = d0;
        cmd_brake        = d1;
        cmd_mode         = d2;
        cmd_angle_tenths = d3;
        useScriptCommand = true;
        // Debug echo so PC can verify Router received the command
        LOG_PORT.print("CMD: speed=");   LOG_PORT.print(cmd_speed_cmPs);
        LOG_PORT.print(" brake=");       LOG_PORT.print(cmd_brake);
        LOG_PORT.print(" mode=");        LOG_PORT.print(cmd_mode);
        LOG_PORT.print(" angle=");       LOG_PORT.println(cmd_angle_tenths);
      }
      // Future: handle other CAN IDs as needed
    } else {
      if (bufIdx < 63) buf[bufIdx++] = c;
    }
  }
}

// ===========================================================================
// CAN frame senders (Router -> bus, called once per loop)
// ===========================================================================

// Note on `f.data` access: the due_can BytesUnion has members
//   uint64_t value;
//   uint32_t low, high;
//   uint16_t s0, s1, s2, s3;
//   uint8_t  bytes[8];
// (no int32/int16 arrays). The Arduino Due is little-endian, so writing the
// uint16/uint32 fields here lays the bytes out LE on the wire as the wiki
// protocol expects. We cast the signed input through the matching unsigned
// type so the bit pattern is preserved without sign-extension surprises.

// 0x4C0 — 8 bytes: east_cm (int32 LE) + north_cm (int32 LE)
bool sendPositionCAN() {
  CAN_FRAME f;
  f.id = CAN_POSITION;
  f.extended = false;
  f.length = 8;
  f.data.low  = (uint32_t)(int32_t)(X_mm / 10);   // east_cm
  f.data.high = (uint32_t)(int32_t)(Y_mm / 10);   // north_cm
  return Can0.sendFrame(f);
}

// 0x4E0 — 2 bytes: heading_centiDeg (int16 LE). Internal storage is tenths-of-deg.
bool sendHeadingCAN() {
  CAN_FRAME f;
  f.id = CAN_HEADING;
  f.extended = false;
  f.length = 2;
  f.data.s0 = (uint16_t)(int16_t)((long)heading_tenths * 10);
  return Can0.sendFrame(f);
}

// 0x4F0 — 2 bytes: speed_cmPs (int16 LE).
bool sendSpeedCAN() {
  CAN_FRAME f;
  f.id = CAN_SPEED;
  f.extended = false;
  f.length = 2;
  f.data.s0 = (uint16_t)(int16_t)(speed_mmPs / 10);
  return Can0.sendFrame(f);
}

// 0x430 — 2 bytes: angle_DegX10 (int16 LE). Substitutes for the L_SENSE/R_SENSE
// analog wires that the bridge harness doesn't carry. DBW reads this every
// loop and uses it as the measured wheel angle for its steering PID. On a
// real trike DBW would read its analog sensors and ignore this frame.
bool sendSteerActualCAN() {
  CAN_FRAME f;
  f.id = CAN_STEER_ACTUAL;
  f.extended = false;
  f.length = 2;
  f.data.s0 = (uint16_t)(int16_t)angle_tenths;
  return Can0.sendFrame(f);
}

// ===========================================================================
// Stage 3: Binary USB packets (kept for the older PC simulator pipe)
// ===========================================================================

// 0x4C0 binary packet to PC via USB. Used by older PC sim that doesn't speak
// CAN; Sensor Hub Nav reads from sendPositionCAN above instead.
// Per wiki: Bytes 1-4 = east_cm (int32), Bytes 5-8 = north_cm (int32)
// Packet header: 2 bytes [ID_HIGH, ID_LOW|(len<<1)]
//   0x4C0 >> 3 = 0x98 -> ID_HIGH
//   ((0x4C0 & 0x07) << 5) | (8 << 1) = 0x00 | 0x10 = 0x10 -> ID_LOW|len
void sendPosition() {
  int32_t east_cm  = (int32_t)(X_mm / 10);
  int32_t north_cm = (int32_t)(Y_mm / 10);

  // CAN ID high byte
  LOG_PORT.write(0x98);
  // CAN ID low bits | data length
  LOG_PORT.write(0x10);
  // east_cm big-endian int32
  LOG_PORT.write((uint8_t)(east_cm >> 24));
  LOG_PORT.write((uint8_t)(east_cm >> 16));
  LOG_PORT.write((uint8_t)(east_cm >>  8));
  LOG_PORT.write((uint8_t)(east_cm      ));
  // north_cm big-endian int32
  LOG_PORT.write((uint8_t)(north_cm >> 24));
  LOG_PORT.write((uint8_t)(north_cm >> 16));
  LOG_PORT.write((uint8_t)(north_cm >>  8));
  LOG_PORT.write((uint8_t)(north_cm      ));
}

// 0x251 GPS origin binary packet. Kept for backwards compat; not called at
// boot in this sketch.
// Per wiki Set Origin (0x251):
//   Byte 1: lat degrees (7 bits) + N/S (bit 8, 0=N)
//   Bytes 2,3,4: lat fraction (0-9,999,999)
//   Byte 5: lon degrees
//   Byte 6 bit 1: E/W (0=E, 1=W); rest of byte 6 + bytes 7,8: lon fraction
void sendOrigin() {
  // Packet header for 0x251, 8 data bytes
  // 0x251 >> 3 = 0x4A -> ID_HIGH
  // ((0x251 & 0x07) << 5) | (8 << 1) = 0x20 | 0x10 = 0x30 -> ID_LOW|len
  LOG_PORT.write(0x4A);
  LOG_PORT.write(0x30);
  // Byte 1: lat degrees, N hemisphere -> bit 8 = 0
  LOG_PORT.write((uint8_t)ORIGIN_LAT);
  // Bytes 2,3,4: lat fraction = 760934
  LOG_PORT.write((uint8_t)(ORIGIN_LAT_FRAC >> 16));
  LOG_PORT.write((uint8_t)(ORIGIN_LAT_FRAC >>  8));
  LOG_PORT.write((uint8_t)(ORIGIN_LAT_FRAC      ));
  // Byte 5: lon degrees = 122
  LOG_PORT.write((uint8_t)ORIGIN_LON);
  // Bytes 6,7,8: W -> first bit of byte 6 = 1, fraction = 189963
  uint32_t lon_field = (1UL << 23) | (uint32_t)ORIGIN_LON_FRAC;
  LOG_PORT.write((uint8_t)(lon_field >> 16));
  LOG_PORT.write((uint8_t)(lon_field >>  8));
  LOG_PORT.write((uint8_t)(lon_field      ));
}
