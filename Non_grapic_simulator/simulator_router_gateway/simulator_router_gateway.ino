/*
 * Simulator — Router Gateway variant (PC-driven closed loop, NO Sensor Hub)

 * Router Arduino Due
 *
 * SISTER SKETCH to ../simulator_closed_loop/simulator_closed_loop.ino.
 *   simulator_closed_loop : Router physics only. Sensor Hub (NavigateTestRunner
 *                           or Navigate) is the PC/CAN gateway that publishes
 *                           0x350 NavDrive + 0x100 NavStatus to DBW.
 *   simulator_router_gateway (this): the Router ALSO plays the gateway role,
 *                           so the test runs on just TWO boards — Router + DBW.
 *
 * The Sensor Hub is removed from this test entirely. The Router:
 *   1. Reads CMD lines from the PC over USB (tools/test_runner.py).
 *   2. Publishes 0x350 NavDrive (the DESIRED speed/brake/angle) + 0x100
 *      NavStatus (auto bit) to DBW every loop — this is the gateway job the
 *      Sensor Hub used to do.
 *   3. Runs the physics: reads DBW's steering pins (D26/D28), integrates the
 *      simulated wheel angle, and publishes 0x430 (the ACTUAL angle) back to
 *      DBW as its PID feedback — plus pose on 0x4C0/0x4E0/0x4F0.
 *   4. Emits a LOG line back to the PC for the scorer.
 *
 *   Inbound  (PC -> Router):  CMD,<id>,<speed>,<brake>,<mode>,<angle>\n
 *   Outbound (Router -> PC):  ACK,<millis>\n
 *                             LOG,<millis>,<key=val>,...\n
 *
 * The command is the DESIRED angle and goes out on 0x350 ONLY. It is never
 * written into angle_tenths — the wheel angle changes only via updateAngle()
 * reading DBW's pins. That long way around (command -> DBW PID -> pins ->
 * physics -> 0x430 -> DBW) is what makes this a real closed-loop test instead
 * of the open-loop simulator_stage1 shortcut.
 *
 * --- Hardware I/O ---
 * Inputs from DBW Arduino (via Bridge harness):
 *   A0  -> THROTTLE   (from DBW DAC0)
 *   D4  -> L_TURN     (from DBW D26): HIGH while DBW wants to turn left
 *   D2  -> R_TURN     (from DBW D28): HIGH while DBW wants to turn right
 * Outputs to DBW Arduino:
 *   D47  -> IRPT_WHEEL (wheel-tick pulse), DAC0/DAC1 -> L_SENSE/R_SENSE
 *
 * CAN frames Router transmits:
 *   0x350  NavDrive    (desired speed/brake/angle) -> DBW       [gateway]
 *   0x100  NavStatus   (auto bit 0x40)             -> DBW       [gateway]
 *   0x430  steer-actual (measured wheel angle)     -> DBW       [feedback]
 *   0x4C0/0x4E0/0x4F0  position / heading / speed
 * CAN frames Router receives:
 *   0x400  Actual (DBW's own reported angle, bytes 4-5) — logged as a second
 *          witness; not required for the loop to close.
 *
 * Shares the integer physics engine via ../libraries/simulator_physics/.
 * Reference: https://www.elcanoproject.org/wiki/Communication
 */

// ============================================================================
// SETUP REQUIRED — set the Arduino IDE Sketchbook Location to the parent of
// this sketch so it finds the shared library:
//   File -> Preferences -> Sketchbook location: <repo>/Non_grapic_simulator
// ============================================================================

#include <SPI.h>
#include <SD.h>
#include <due_can.h>

// ===== Logging / USB Serial port =====
// SerialUSB (Native USB jack) keeps D0/D1 quiet so we don't garbage DBW's
// inter-board UART on the Bridge harness. The PC test runner talks here too.
#define USE_NATIVE_USB

#ifdef USE_NATIVE_USB
  #define LOG_PORT SerialUSB
#else
  #define LOG_PORT Serial
#endif

// ===== Pin Definitions =====
#define THROTTLE_PIN    A0
#define BRAKE_VOLT_PIN  42
#define BRAKE_ON_PIN    48
// Steering inputs from DBW (old two-pin L_TURN/R_TURN scheme):
//   L_TURN_PIN (D4) <- DBW D26 : HIGH while DBW wants to turn left
//   R_TURN_PIN (D2) <- DBW D28 : HIGH while DBW wants to turn right
#define L_TURN_PIN      4
#define R_TURN_PIN      2
#define IRPT_WHEEL_PIN  47
#define L_SENSE_PIN     DAC0
#define R_SENSE_PIN     DAC1

// ===== SD Card Pin =====
#define SD_CS_PIN       37

// ===== CAN ID Definitions =====
// Per https://www.elcanoproject.org/wiki/Communication
#define NavDrive_CANID    0x350   // Router->DBW: desired speed/brake/angle (gateway)
#define NavStatus_CANID   0x100   // Router->DBW: status byte, 0x40 = autonomous
#define Actual_CANID      0x400   // DBW->Router: actual speed + wheel angle
#define CAN_POSITION      0x4C0   // Router->bus: east_cm, north_cm
#define CAN_HEADING       0x4E0   // Router->bus: heading_centiDeg
#define CAN_SPEED         0x4F0   // Router->bus: speed_cmPs
#define CAN_STEER_ACTUAL  0x430   // Router->DBW: simulated actual wheel angle (degx10)

// ===== Speed Model Constants =====
#define FRICTION_NUM            9296
#define FRICTION_DEN            10000
#define MIN_EFFECTIVE_THROTTLE  65
#define MAX_EFFECTIVE_THROTTLE  227
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
int angle_tenths   = 0;     // ACTUAL simulated wheel angle (driven by DBW pins)

long X_mm = 0;
long Y_mm = 0;

unsigned long nextPulseTime_ms = 0;

// ===== SD Card Variables =====
File logFile;
bool sdAvailable = false;

// ===== Gateway: latched command from PC =====
// Republished on 0x350 every loop once the first CMD arrives. brake defaults
// to 100 (engaged) so DBW holds still if a test forgets brake=0 before speed>0.
int16_t cmd_speed_cmPs   = 0;
int16_t cmd_brake        = 100;
uint8_t cmd_mode         = 0;
int16_t cmd_angle_DegX10 = 0;     // DESIRED angle — goes to DBW on 0x350 only
bool    cmdLatched       = false;
char    cmdBuf[64];
int     cmdBufIdx        = 0;

// DBW's own reported angle (0x400 bytes 4-5) — second witness for the LOG.
int16_t dbwAngle_DegX10  = 0;

// ===== Function Declarations =====
bool sendPositionCAN();
bool sendHeadingCAN();
bool sendSpeedCAN();
bool sendSteerActualCAN();
void publishCmdFrames();
void readTestCommand();
void drainCanRx();
void printTestLog();

// Shared physics engine (sin1000, cos1000, computeSpeed, updateAngle,
// updatePosition, sendWheelPulse, sendAngleSensor). Included AFTER the globals
// above because its inline bodies reference them by name.
#include <simulator_physics.h>

// ===========================================================================
// Setup
// ===========================================================================
void setup() {
  LOG_PORT.begin(115200);
#ifdef USE_NATIVE_USB
  uint32_t waitStart = millis();
  while (!LOG_PORT && (millis() - waitStart) < 3000);
#endif

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(BRAKE_VOLT_PIN, INPUT);
  pinMode(BRAKE_ON_PIN, INPUT);
  pinMode(L_TURN_PIN, INPUT);
  pinMode(R_TURN_PIN, INPUT);
  pinMode(IRPT_WHEEL_PIN, OUTPUT);
  digitalWrite(IRPT_WHEEL_PIN, LOW);

  // Due DAC is 12-bit; default analogWrite is 8-bit and would truncate the
  // L_SENSE/R_SENSE outputs in sendAngleSensor.
  analogWriteResolution(12);

  for (int i = 0; i < THROTTLE_HISTORY; i++) throttleHistory[i] = 0;

  // CAN at 500 kbps. Unlike simulator_closed_loop, this sketch RECEIVES too
  // (0x400 from DBW), so install a catch-all RX filter.
  if (Can0.begin(CAN_BPS_500K)) {
    Can0.watchFor();
    LOG_PORT.println("CAN init OK");
  } else {
    LOG_PORT.println("CAN init FAILED");
  }

  // Initialize SD card (diagnostic CSV only — the PC scorer uses the LOG line).
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(100);
  if (SD.begin(SD_CS_PIN)) {
    char filename[13];
    for (int i = 0; i < 100; i++) {
      sprintf(filename, "SIM%02d.CSV", i);
      if (!SD.exists(filename)) break;
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
      sdAvailable = true;
      logFile.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,cmd_angle,brakeOn");
      logFile.flush();
    }
  }

  // Boot banner the PC test runner can watch for.
  LOG_PORT.println("RDY,router_gateway_v0");
}

// ===========================================================================
// Main Loop
// ===========================================================================
void loop() {
  uint32_t startTime = millis();

  drainCanRx();        // capture DBW's 0x400 actual angle (second witness)
  readTestCommand();   // latch any PC CMD as the desired command

  // --- Physics: read DBW outputs, integrate the simulated wheel ---
  // THROTTLE closes the speed loop through DBW: 0x350 speed -> DBW throttle PID
  // -> DBW DAC0 -> A0 here.
  int rawThrottle = analogRead(THROTTLE_PIN);
  int throttle = rawThrottle / 4;
  bool brakeOn = (throttle < MIN_EFFECTIVE_THROTTLE);

  // Steering pins from DBW (two-wire L_TURN/R_TURN). The wheel angle is driven
  // ONLY here — never written directly from the command.
  bool lTurn = (digitalRead(L_TURN_PIN) == HIGH);
  bool rTurn = (digitalRead(R_TURN_PIN) == HIGH);
  angle_tenths = updateAngle(lTurn, rTurn);

  throttleHistory[historyIndex] = throttle;
  historyIndex = (historyIndex + 1) % THROTTLE_HISTORY;
  speed_mmPs   = computeSpeed(throttle, brakeOn);

  updatePosition(speed_mmPs, heading_tenths, angle_tenths);

  // --- Simulated sensor outputs to DBW ---
  sendWheelPulse(speed_mmPs);
  sendAngleSensor(angle_tenths);

  // --- CAN out: pose + actual wheel angle (DBW's PID feedback) ---
  sendPositionCAN();
  sendHeadingCAN();
  sendSpeedCAN();
  sendSteerActualCAN();

  // --- Gateway: once a command has arrived, drive DBW + report to the PC ---
  if (cmdLatched) {
    publishCmdFrames();   // 0x350 desired + 0x100 auto, every loop
    printTestLog();       // LOG line for the scorer
  }

  // --- Diagnostic CSV to SD only (keep USB clean for the LOG stream) ---
  if (sdAvailable) {
    logFile.print(millis());       logFile.print(",");
    logFile.print(X_mm);           logFile.print(",");
    logFile.print(Y_mm);           logFile.print(",");
    logFile.print(heading_tenths); logFile.print(",");
    logFile.print(speed_mmPs);     logFile.print(",");
    logFile.print(angle_tenths);   logFile.print(",");
    logFile.print(cmd_angle_DegX10); logFile.print(",");
    logFile.println(brakeOn ? 1 : 0);
    logFile.flush();
  }

  uint32_t elapsed = millis() - startTime;
  if (elapsed < LOOP_TIME_MS) delay(LOOP_TIME_MS - elapsed);
}

// ===========================================================================
// Gateway helpers
// ===========================================================================

// Parse CMD,<id>,<speed>,<brake>,<mode>,<angle> from the PC over USB.
void readTestCommand() {
  while (LOG_PORT.available() > 0) {
    char c = (char)LOG_PORT.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdBufIdx] = '\0';
      int id, speed, brake, mode, angle;
      if (cmdBufIdx > 4 &&
          sscanf(cmdBuf, "CMD,%i,%i,%i,%i,%i",
                 &id, &speed, &brake, &mode, &angle) == 5) {
        cmd_speed_cmPs   = (int16_t)speed;
        cmd_brake        = (int16_t)brake;
        cmd_mode         = (uint8_t)mode;
        cmd_angle_DegX10 = (int16_t)angle;
        cmdLatched       = true;
        LOG_PORT.print("ACK,"); LOG_PORT.println(millis());
      }
      cmdBufIdx = 0;
    } else if (cmdBufIdx < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdBufIdx++] = c;
    }
  }
}

// Publish the latched command as 0x350 NavDrive + 0x100 NavStatus.
// Layout per DBW Vehicle.cpp: bytes 0-1 speed int16, byte 2 brake, byte 3 mode,
// bytes 4-5 angle int16. (s0/s1/s2 are the 16-bit lanes of the due_can union.)
void publishCmdFrames() {
  CAN_FRAME f;
  f.extended = false;

  f.id = NavDrive_CANID;
  f.length = 6;
  f.data.s0 = (uint16_t)cmd_speed_cmPs;     // bytes 0-1
  f.data.s1 = (uint16_t)cmd_brake;          // byte 2 (brake), byte 3 (mode=0)
  f.data.s2 = (uint16_t)cmd_angle_DegX10;   // bytes 4-5
  Can0.sendFrame(f);

  f.id = NavStatus_CANID;
  f.length = 1;
  f.data.bytes[0] = 0x40;                   // autonomous bit -> DBW obeys 0x350
  Can0.sendFrame(f);
}

// Drain RX: capture DBW's reported actual angle from 0x400 (bytes 4-5).
void drainCanRx() {
  CAN_FRAME in;
  while (Can0.available() > 0) {
    Can0.read(in);
    if (in.id == Actual_CANID && in.length >= 6) {
      dbwAngle_DegX10 = (int16_t)in.data.s2;
    }
  }
}

// One machine-parseable LOG line for the PC scorer. actual_angle_tenths is the
// simulated wheel angle (what we publish on 0x430, what the asserts check);
// dbw_angle_tenths is DBW's own echo for cross-checking.
void printTestLog() {
  LOG_PORT.print("LOG,");                    LOG_PORT.print(millis());
  LOG_PORT.print(",east_cm=");               LOG_PORT.print(X_mm / 10);
  LOG_PORT.print(",north_cm=");              LOG_PORT.print(Y_mm / 10);
  LOG_PORT.print(",heading_centiDeg=");      LOG_PORT.print(heading_tenths * 10);
  LOG_PORT.print(",actual_angle_tenths=");   LOG_PORT.print(angle_tenths);
  LOG_PORT.print(",dbw_angle_tenths=");      LOG_PORT.print(dbwAngle_DegX10);
  LOG_PORT.print(",cmd_angle_tenths=");      LOG_PORT.print(cmd_angle_DegX10);
  LOG_PORT.print(",sim_speed_cmPs=");        LOG_PORT.print(speed_mmPs / 10);
  LOG_PORT.print(",cmd_speed_cmPs=");        LOG_PORT.println(cmd_speed_cmPs);
}

// ===========================================================================
// CAN frame senders (Router -> bus, once per loop). The due_can BytesUnion has
// value / low,high / s0,s1,s2,s3 / bytes[8] — no int16/int32 arrays. The Due is
// little-endian so these lay out LE on the wire as the wiki protocol expects.
// ===========================================================================

// 0x4C0 — 8 bytes: east_cm (int32 LE) + north_cm (int32 LE)
bool sendPositionCAN() {
  CAN_FRAME f;
  f.id = CAN_POSITION;
  f.extended = false;
  f.length = 8;
  f.data.low  = (uint32_t)(int32_t)(X_mm / 10);
  f.data.high = (uint32_t)(int32_t)(Y_mm / 10);
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

// 0x430 — 2 bytes: angle_DegX10 (int16 LE). DBW reads this as its measured
// wheel angle for the steering PID.
bool sendSteerActualCAN() {
  CAN_FRAME f;
  f.id = CAN_STEER_ACTUAL;
  f.extended = false;
  f.length = 2;
  f.data.s0 = (uint16_t)(int16_t)angle_tenths;
  return Can0.sendFrame(f);
}
