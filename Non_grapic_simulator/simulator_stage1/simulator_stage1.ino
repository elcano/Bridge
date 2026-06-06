/*
 * Simulator Stage 1 + Stage 3
 * Router Arduino Due
 *
 * Reads inputs from DBW Arduino OR from PC via USB Serial (Stage 2 simulator):
 *   A0  -> THROTTLE  (from DBW, if connected)
 *   D42 -> BRAKE_VOLT
 *   D48 -> BRAKE_ON
 *   D4  -> L_TURN
 *   D2  -> R_TURN
 *
 * Outputs to DBW Arduino:
 *   D47  -> IRPT_WHEEL (wheel pulse)
 *   DAC0 -> L_SENSE (left wheel angle)
 *   DAC1 -> R_SENSE (right wheel angle)
 *
 * Stage 2 integration (PC -> Router):
 *   Reads ASCII commands from USB Serial sent by PC simulator.
 *   Format: "CANID,nbytes,data1,data2,...\n"
 *   Example: "350,6,1500,0,1,-21"
 *     CANID=350, nbytes=6
 *     data: speed=1500 cm/s, brake=0, mode=1, angle=-21 (=-2.1 deg)
 *   Lines starting with '#' are comments and ignored.
 *   Optional execution time prefix: "1000,350,6,1500,0,1,-21"
 *
 * Stage 3 GPS output (Router -> PC):
 *   Sends binary packets over USB Serial every loop.
 *   0x251: GPS origin (UW Bothell) once at startup.
 *   0x4C0: Vehicle position (east_cm, north_cm) every 100ms.
 *   Binary format per professor's instruction:
 *     same data as CAN, encoding is binary.
 *
 * Logs CSV to SD card if available, falls back to SerialUSB.
 * CSV: time_ms, X_mm, Y_mm, heading_tenths, speed_mmPs, angle_tenths, throttle, brakeOn
 *
 * All arithmetic uses integers (no float) per professor's requirement.
 * Angles stored as tenths of a degree. Positions in mm. Speed in mm/s.
 *
 * Reference: https://www.elcanoproject.org/wiki/Communication
 */

#include <SPI.h>
#include <SD.h>
#include <due_can.h>

// ===== Pin Definitions =====
#define THROTTLE_PIN    A0   // From DBW DAC0
#define BRAKE_VOLT_PIN  42   // From DBW D40
#define BRAKE_ON_PIN    48   // From DBW D44
// TWO-WIRE DIGITAL STEERING (per Minhee's bridge wiring):
//   Router D4 (L_TURN) ← DBW D26 : HIGH while DBW wants to turn left
//   Router D2 (R_TURN) ← DBW D28 : HIGH while DBW wants to turn right
// Read with digitalRead; updateAngle() bumps angle_tenths each loop while
// a direction is asserted, and writes the new angle back as voltage on
// DAC0/DAC1 (L_SENSE/R_SENSE) so DBW can close its loop.
#define L_TURN_PIN      4
#define R_TURN_PIN      2
#define IRPT_WHEEL_PIN  47   // To DBW D47
#define L_SENSE_PIN     DAC0 // To DBW A10
#define R_SENSE_PIN     DAC1 // To DBW A11

// ===== SD Card Pin =====
#define SD_CS_PIN       37

// ===== CAN ID Definitions =====
// Per https://www.elcanoproject.org/wiki/Communication
#define CAN_DRIVE         0x350  // Nav->DBW: speed(cm/s), brake, mode, angle(deg x10)
#define CAN_POSITION      0x4C0  // Router->Sensor Hub: vehicle position east_cm, north_cm
#define CAN_HEADING       0x4E0  // Router->Sensor Hub: heading_centiDeg (compass only)
#define CAN_SPEED         0x4F0  // Router->Sensor Hub: speed_cmPs (velocity only, separate frame)
#define CAN_STEER_ACTUAL  0x430  // Router->DBW: simulated actual wheel angle (deg x10).
                                 // Substitutes for L_SENSE/R_SENSE analog feedback that
                                 // is unavailable on this bridge. On the real trike, DBW
                                 // would read its analog sensors and ignore this frame.
#define CAN_ORIGIN        0x251  // Nav->all: GPS origin lat/lon

// ===== Origin GPS coordinates (UW Bothell) =====
#define ORIGIN_LAT       47      // latitude degrees
#define ORIGIN_LAT_FRAC  760934  // latitude fraction (0-9,999,999)
#define ORIGIN_LON       122     // longitude degrees
#define ORIGIN_LON_FRAC  189963  // longitude fraction (0-999,999), West

// ===== Speed Model Constants =====
#define FRICTION_NUM            9296
#define FRICTION_DEN            10000
#define MIN_EFFECTIVE_THROTTLE  65
#define MAX_EFFECTIVE_THROTTLE  227
#define MAX_SPEED_mmPs          2000   // 2 m/s cap — slow enough that turn radius (~3 m) fits within waypoint radius (3 m). Was 13600 (13.6 m/s) but turn radius scaled too wide.
#define THROTTLE_HISTORY        10
#define THROTTLE_DELAY_START    3
#define THROTTLE_DELAY_END      10

// ===== Vehicle Settings =====
#define WHEEL_DIAMETER_MM    495
#define WHEEL_CIRCUM_MM      1555
#define LOOP_TIME_MS         100
#define MAX_ANGLE_TENTHS     250
// Restored to 20 (200 degX10/sec slew). This now represents realistic steer
// motor inertia — the motor takes ~1.25s to traverse full range. DBW closes
// the loop against the measured wheel angle (published over CAN 0x430), so
// the per-loop slew rate no longer matters for stability — DBW drives the
// motor only while measured < desired, then stops.
#define ANGLE_CHANGE_TENTHS  20

// ===== Wheel Angle Sensor =====
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
int angle_tenths           = 0;   // actual wheel angle (physics state)
int commanded_angle_tenths = 0;   // desired angle from DBW (decoded from PWM)

long X_mm = 0;
long Y_mm = 0;

unsigned long nextPulseTime_ms = 0;

// ===== Stage 2: ASCII command state =====
bool useScriptCommand  = false;
int  cmd_speed_cmPs    = 0;  // commanded speed from PC (cm/s)
int  cmd_brake         = 0;  // 0=off, 1=hold(12V), 2=on(24V)
int  cmd_mode          = 0;  // 0=init,1=RC,2=operator,3=auto-RC,4=auto-op
int  cmd_angle_tenths  = 0;  // commanded steer angle (degrees x10)

// ===== SD Card Variables =====
File logFile;
bool sdAvailable = false;

// ===== Function Declarations =====
int  computeSpeed(int throttle, bool brakeOn);
int  updateAngle(bool lTurn, bool rTurn);
void updatePosition(int speed, int &heading, int angle);
void sendWheelPulse(int speed);
void sendAngleSensor(int angleT);
void readScriptCommand();
void sendPosition();
void sendOrigin();
bool sendPositionCAN();
bool sendHeadingCAN();
bool sendSpeedCAN();
bool sendSteerActualCAN();

// ===== Integer sine/cosine (returns value * 1000) =====
int sin1000(int angle_tenths) {
  angle_tenths = ((angle_tenths % 3600) + 3600) % 3600;
  static const int sinTable[91] = {
    0, 17, 35, 52, 70, 87, 105, 122, 139, 156,
    174, 191, 208, 225, 242, 259, 276, 292, 309, 326,
    342, 358, 375, 391, 407, 423, 438, 454, 469, 485,
    500, 515, 530, 545, 559, 574, 588, 602, 616, 629,
    643, 656, 669, 682, 695, 707, 719, 731, 743, 755,
    766, 777, 788, 799, 809, 819, 829, 839, 848, 857,
    866, 875, 883, 891, 899, 906, 914, 921, 927, 934,
    940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
    985, 988, 990, 993, 995, 996, 998, 999, 999, 1000,
    1000
  };
  int deg = angle_tenths / 10;
  if (angle_tenths < 900)       return sinTable[deg];
  else if (angle_tenths < 1800) return sinTable[180 - deg];
  else if (angle_tenths < 2700) return -sinTable[deg - 180];
  else                          return -sinTable[360 - deg];
}

int cos1000(int angle_tenths) {
  return sin1000(angle_tenths + 900);
}

// ===== Setup =====
void setup() {
  SerialUSB.begin(115200);
  // Bounded wait so the sketch boots even if no PC is attached to Native USB.
  uint32_t waitStart = millis();
  while (!SerialUSB && (millis() - waitStart) < 3000);

  // Configure pins
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(BRAKE_VOLT_PIN, INPUT);
  pinMode(BRAKE_ON_PIN, INPUT);
  pinMode(L_TURN_PIN, INPUT);
  pinMode(R_TURN_PIN, INPUT);
  // Explicitly disable the SAM3X internal pullup on the L_TURN/R_TURN inputs.
  // Without this, the pullup state at boot is non-deterministic, which can
  // cause digitalRead to return HIGH even when DBW is actively pulling LOW.
  // The isolated wire test (WireTest_Router.ino) does this and reads clean
  // values; the production sketch was not doing it.
  g_APinDescription[L_TURN_PIN].pPort->PIO_PUDR = g_APinDescription[L_TURN_PIN].ulPin;
  g_APinDescription[R_TURN_PIN].pPort->PIO_PUDR = g_APinDescription[R_TURN_PIN].ulPin;
  pinMode(IRPT_WHEEL_PIN, OUTPUT);
  digitalWrite(IRPT_WHEEL_PIN, LOW);

  // Due DAC is 12-bit (0-4095). Default analogWrite resolution is 8-bit,
  // which truncates the lSense*4 / rSense*4 values written below and
  // saturates DBW's A10/A11 readings. Set 12-bit so calibration matches.
  analogWriteResolution(12);

  for (int i = 0; i < THROTTLE_HISTORY; i++) throttleHistory[i] = 0;

  // Initialize CAN bus at 500 kbps (matches DBW + Sensor Hub).
  // Router only transmits (position, heading, speed, steer-actual); it does
  // not call Can0.read() anywhere, so no RX mailboxes are needed. All 8
  // SAM3X mailboxes stay available for TX.
  if (Can0.begin(CAN_BPS_500K)) {
    SerialUSB.println("CAN init OK");
  } else {
    SerialUSB.println("CAN init FAILED");
  }

  // (Original sendOrigin() binary USB packet removed — position/heading now go
  //  out via CAN 0x4C0/0x4E0 instead of binary USB packets to a PC.)

  // Initialize SD card
  SerialUSB.print("Initializing SD card on pin D");
  SerialUSB.print(SD_CS_PIN);
  SerialUSB.print("... ");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(100);
  if (SD.begin(SD_CS_PIN)) {
    SerialUSB.println("SD card found!");
    char filename[13];
    for (int i = 0; i < 100; i++) {
      sprintf(filename, "SIM%02d.CSV", i);
      if (!SD.exists(filename)) break;
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
      sdAvailable = true;
      SerialUSB.print("Logging to SD: ");
      SerialUSB.println(filename);
    } else {
      SerialUSB.println("Failed to open file. Using SerialUSB.");
    }
  } else {
    SerialUSB.println("SD not found. Using SerialUSB.");
  }

  if (sdAvailable) {
    logFile.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,throttle,brakeOn");
    logFile.flush();
  } else {
    SerialUSB.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,throttle,brakeOn");
  }
  SerialUSB.println("Simulator Stage 1+3 started.");

  // Boot-time self-diagnostic: sample L_TURN/R_TURN inputs at 5 Hz for 5 s
  // before entering the main loop. CAN is initialized but no traffic yet
  // (Sensor Hub hasn't started broadcasting if booted together). Use this
  // to see whether the pullup-disable above is honored at boot, and what
  // the floating baseline reads.
  SerialUSB.println("# boot diag (5s): t_ms,L,R");
  uint32_t diagStart = millis();
  while (millis() - diagStart < 5000) {
    SerialUSB.print(millis()); SerialUSB.print(",");
    SerialUSB.print(digitalRead(L_TURN_PIN)); SerialUSB.print(",");
    SerialUSB.println(digitalRead(R_TURN_PIN));
    delay(200);
  }
  SerialUSB.println("# boot diag done — entering main loop");
}

// ===== Main Loop =====
void loop() {
  // (Original 50-second auto-halt removed — Router now runs continuously
  //  so Sensor Hub keeps receiving 0x4C0/0x4E0 frames.)
  uint32_t startTime = millis();

  // Stage 2: read ASCII command from PC if available
  readScriptCommand();

  int throttle;
  bool brakeOn;
  bool lTurn;
  bool rTurn;

  if (useScriptCommand) {
    // Use drive command received from PC simulator (Stage 2)
    // speed and angle are set directly; skip throttle model
    speed_mmPs   = cmd_speed_cmPs * 10;  // cm/s -> mm/s
    angle_tenths = cmd_angle_tenths;
    brakeOn      = (cmd_brake > 0);
    throttle     = 0;
    lTurn        = false;
    rTurn        = false;
  } else {
    // Read real inputs from DBW (closes the loop: Nav -> CAN -> DBW -> pins -> here).
    //   THROTTLE_PIN (A0)    <- DBW DAC0 throttle output, 0-1023 analog
    //   BRAKE_ON_PIN (D48)   <- DBW BRAKE_ON_PIN (D44), INVERTED
    //                           (DBW Settings.h RELAYInversion=true:
    //                            HIGH=released, LOW=engaged)
    //   L_TURN_PIN  (D4)     <- DBW LEFT_TURN_PIN  (D26): HIGH = turn left
    //   R_TURN_PIN  (D2)     <- DBW RIGHT_TURN_PIN (D28): HIGH = turn right
    int rawThrottle = analogRead(THROTTLE_PIN);
    throttle = rawThrottle / 4;
    // BRAKE: derived from throttle below the effective threshold. When Nav
    // commands speed=0, DBW's throttle PID outputs 0 and we treat that as
    // brake. The DBW brake-relay wire is also available now (BRAKE_ON_PIN D48)
    // if we want to switch to digitalRead(BRAKE_ON_PIN) and invert.
    brakeOn  = (throttle < MIN_EFFECTIVE_THROTTLE);

    // STEERING via two digital wires from DBW (L_TURN D4, R_TURN D2).
    // digitalRead is a single PIO register access — robust against CAN
    // interrupt contention, which we suspect was disrupting the previous
    // pulseIn-based PWM scheme under heavy bus load.
    lTurn = (digitalRead(L_TURN_PIN) == HIGH);
    rTurn = (digitalRead(R_TURN_PIN) == HIGH);
    angle_tenths = updateAngle(lTurn, rTurn);
    static int dbgCount = 0;
    if (++dbgCount >= 10) {
      dbgCount = 0;
      SerialUSB.print("# L="); SerialUSB.print(lTurn ? 1 : 0);
      SerialUSB.print(" R="); SerialUSB.print(rTurn ? 1 : 0);
      SerialUSB.print(" ang_tenths="); SerialUSB.print(angle_tenths);
      SerialUSB.print(" speed="); SerialUSB.println(speed_mmPs);
    }

    throttleHistory[historyIndex] = throttle;
    historyIndex = (historyIndex + 1) % THROTTLE_HISTORY;
    speed_mmPs   = computeSpeed(throttle, brakeOn);
  }

  // Update vehicle position
  updatePosition(speed_mmPs, heading_tenths, angle_tenths);

  // Send simulated sensor values to DBW Arduino
  sendWheelPulse(speed_mmPs);
  sendAngleSensor(angle_tenths);

  // Broadcast position, heading, and speed on CAN — each its own separate frame.
  // (Binary USB sendPosition() removed — was garbling the Serial Monitor display.)
  static uint32_t txOk = 0, txFail = 0;
  if (sendPositionCAN())    txOk++; else txFail++;
  if (sendHeadingCAN())     txOk++; else txFail++;
  if (sendSpeedCAN())       txOk++; else txFail++;
  if (sendSteerActualCAN()) txOk++; else txFail++;
  static uint32_t lastTxDbg_ms = 0;
  if (millis() - lastTxDbg_ms > 2000) {
    lastTxDbg_ms = millis();
    SerialUSB.print("# Router TX txOk="); SerialUSB.print(txOk);
    SerialUSB.print(" txFail="); SerialUSB.println(txFail);
  }

  // Log CSV to SD card or Serial
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
    SerialUSB.print(millis());       SerialUSB.print(",");
    SerialUSB.print(X_mm);           SerialUSB.print(",");
    SerialUSB.print(Y_mm);           SerialUSB.print(",");
    SerialUSB.print(heading_tenths); SerialUSB.print(",");
    SerialUSB.print(speed_mmPs);     SerialUSB.print(",");
    SerialUSB.print(angle_tenths);   SerialUSB.print(",");
    SerialUSB.print(throttle);       SerialUSB.print(",");
    SerialUSB.println(brakeOn ? 1 : 0);
  }

  uint32_t elapsed = millis() - startTime;
  if (elapsed < LOOP_TIME_MS) delay(LOOP_TIME_MS - elapsed);
}

// ===== Stage 2: Read ASCII drive command from PC via USB Serial =====
// Format: "CANID,nbytes,data1,data2,...\n"
// Example: "350,6,1500,0,1,-21"
//   CAN ID 0x350, 6 data bytes
//   speed=1500 cm/s, brake=0, mode=1, angle=-21 (=-2.1 deg left)
// Optional execution time prefix (ms): "1000,350,6,1500,0,1,-21"
// Lines starting with '#' are comments and ignored.
void readScriptCommand() {
  static char buf[64];
  static int  bufIdx = 0;

  while (SerialUSB.available()) {
    char c = (char)SerialUSB.read();
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
      int d0 = (nFields > idx+2) ? fields[idx+2] : 0;  // speed (cm/s)
      int d1 = (nFields > idx+3) ? fields[idx+3] : 0;  // brake
      int d2 = (nFields > idx+4) ? fields[idx+4] : 0;  // mode
      int d3 = (nFields > idx+5) ? fields[idx+5] : 0;  // angle (deg x10)

      if (canId == CAN_DRIVE) {
        // 0x350: speed(cm/s), brake, mode, steer angle(deg x10)
        cmd_speed_cmPs   = d0;
        cmd_brake        = d1;
        cmd_mode         = d2;
        cmd_angle_tenths = d3;
        useScriptCommand = true;
        // Debug print so PC can verify Router received the command
        SerialUSB.print("CMD: speed=");   SerialUSB.print(cmd_speed_cmPs);
        SerialUSB.print(" brake=");       SerialUSB.print(cmd_brake);
        SerialUSB.print(" mode=");        SerialUSB.print(cmd_mode);
        SerialUSB.print(" angle=");       SerialUSB.println(cmd_angle_tenths);
      }
      // Future: handle other CAN IDs as needed
    } else {
      if (bufIdx < 63) buf[bufIdx++] = c;
    }
  }
}

// ===== Stage 3: Send vehicle position as binary 0x4C0 packet =====
// Per wiki: Bytes 1-4 = east_cm (int32), Bytes 5-8 = north_cm (int32)
// Packet header: 2 bytes [ID_HIGH, ID_LOW|(len<<1)]
//   0x4C0 >> 3 = 0x98 -> ID_HIGH
//   ((0x4C0 & 0x07) << 5) | (8 << 1) = 0x00 | 0x10 = 0x10 -> ID_LOW|len
void sendPosition() {
  int32_t east_cm  = (int32_t)(X_mm / 10);
  int32_t north_cm = (int32_t)(Y_mm / 10);

  SerialUSB.write(0x98);  // CAN ID high byte
  SerialUSB.write(0x10);  // CAN ID low bits | data length
  // east_cm big-endian int32
  SerialUSB.write((uint8_t)(east_cm >> 24));
  SerialUSB.write((uint8_t)(east_cm >> 16));
  SerialUSB.write((uint8_t)(east_cm >>  8));
  SerialUSB.write((uint8_t)(east_cm      ));
  // north_cm big-endian int32
  SerialUSB.write((uint8_t)(north_cm >> 24));
  SerialUSB.write((uint8_t)(north_cm >> 16));
  SerialUSB.write((uint8_t)(north_cm >>  8));
  SerialUSB.write((uint8_t)(north_cm      ));
}

// ===== Send vehicle position on real CAN bus (0x4C0) =====
// 8 bytes: east_cm (int32 LE) + north_cm (int32 LE)
bool sendPositionCAN() {
  CAN_FRAME f;
  f.id = CAN_POSITION;
  f.extended = false;
  f.length = 8;
  f.data.int32[0] = (int32_t)(X_mm / 10);   // east_cm
  f.data.int32[1] = (int32_t)(Y_mm / 10);   // north_cm
  return Can0.sendFrame(f);
}

// ===== Send vehicle heading on real CAN bus (0x4E0) — heading only =====
// 2 bytes: heading_centiDeg (int16 LE)
// Internal heading is in tenths of a degree; convert to centidegrees (×10).
bool sendHeadingCAN() {
  CAN_FRAME f;
  f.id = CAN_HEADING;
  f.extended = false;
  f.length = 2;
  f.data.int16[0] = (int16_t)((long)heading_tenths * 10);
  return Can0.sendFrame(f);
}

// ===== Send vehicle speed on real CAN bus (0x4F0) — speed only =====
// 2 bytes: speed_cmPs (int16 LE)
bool sendSpeedCAN() {
  CAN_FRAME f;
  f.id = CAN_SPEED;
  f.extended = false;
  f.length = 2;
  f.data.int16[0] = (int16_t)(speed_mmPs / 10);
  return Can0.sendFrame(f);
}

// ===== Send simulated actual wheel angle on CAN bus (0x430) =====
// 2 bytes: angle_DegX10 (int16 LE), range ±MAX_ANGLE_TENTHS (±250 = ±25°).
// Substitutes for the L_SENSE/R_SENSE analog feedback that the bridge wires
// don't carry. DBW reads this every loop and uses it as the measured wheel
// angle for its steering PID. On the real trike, this frame would not be
// transmitted, and DBW would fall back to L_SENSE/R_SENSE analog reads.
bool sendSteerActualCAN() {
  CAN_FRAME f;
  f.id = CAN_STEER_ACTUAL;
  f.extended = false;
  f.length = 2;
  f.data.int16[0] = (int16_t)angle_tenths;
  return Can0.sendFrame(f);
}

// ===== Stage 3: Send GPS origin as binary 0x251 packet =====
// Per wiki Set Origin (0x251):
//   Byte 1: lat degrees (7 bits) + N/S (bit 8, 0=N)
//   Bytes 2,3,4: lat fraction (0-9,999,999)
//   Byte 5: lon degrees
//   Byte 6 bit 1: E/W (0=E, 1=W); rest of byte 6 + bytes 7,8: lon fraction
// Sent once at startup.
void sendOrigin() {
  // Packet header for 0x251, 8 data bytes
  // 0x251 >> 3 = 0x4A -> ID_HIGH
  // ((0x251 & 0x07) << 5) | (8 << 1) = 0x20 | 0x10 = 0x30 -> ID_LOW|len
  SerialUSB.write(0x4A);
  SerialUSB.write(0x30);
  // Byte 1: lat degrees, N hemisphere -> bit 8 = 0
  SerialUSB.write((uint8_t)ORIGIN_LAT);
  // Bytes 2,3,4: lat fraction = 760934
  SerialUSB.write((uint8_t)(ORIGIN_LAT_FRAC >> 16));
  SerialUSB.write((uint8_t)(ORIGIN_LAT_FRAC >>  8));
  SerialUSB.write((uint8_t)(ORIGIN_LAT_FRAC      ));
  // Byte 5: lon degrees = 122
  SerialUSB.write((uint8_t)ORIGIN_LON);
  // Bytes 6,7,8: W -> first bit of byte 6 = 1, fraction = 189963
  uint32_t lon_field = (1UL << 23) | (uint32_t)ORIGIN_LON_FRAC;
  SerialUSB.write((uint8_t)(lon_field >> 16));
  SerialUSB.write((uint8_t)(lon_field >>  8));
  SerialUSB.write((uint8_t)(lon_field      ));
}

// ===== Compute Speed (integer arithmetic) =====
int computeSpeed(int throttle, bool brakeOn) {
  if (brakeOn) { prevSpeed_mmPs = 0; return 0; }
  long sum = 0;
  for (int i = THROTTLE_DELAY_START; i <= THROTTLE_DELAY_END; i++) {
    int idx = (historyIndex - i + THROTTLE_HISTORY) % THROTTLE_HISTORY;
    sum += throttleHistory[idx];
  }
  int meanThrottle = (int)(sum / 8);
  int estimatedSpeed = 0;
  if (meanThrottle > MIN_EFFECTIVE_THROTTLE)
    estimatedSpeed = (int)((long)MAX_SPEED_mmPs *
                    (meanThrottle - MIN_EFFECTIVE_THROTTLE) /
                    (MAX_EFFECTIVE_THROTTLE - MIN_EFFECTIVE_THROTTLE));
  if (estimatedSpeed < 0) estimatedSpeed = 0;
  int momentum = (int)((long)prevSpeed_mmPs * FRICTION_NUM / FRICTION_DEN);
  int newSpeed = (momentum > estimatedSpeed) ? momentum : estimatedSpeed;
  if (newSpeed > MAX_SPEED_mmPs) newSpeed = MAX_SPEED_mmPs;
  prevSpeed_mmPs = newSpeed;
  return newSpeed;
}

// ===== Update Steering Angle =====
int updateAngle(bool lTurn, bool rTurn) {
  if (lTurn && !rTurn)      angle_tenths -= ANGLE_CHANGE_TENTHS;
  else if (!lTurn && rTurn) angle_tenths += ANGLE_CHANGE_TENTHS;
  if (angle_tenths > MAX_ANGLE_TENTHS)  angle_tenths = MAX_ANGLE_TENTHS;
  if (angle_tenths < -MAX_ANGLE_TENTHS) angle_tenths = -MAX_ANGLE_TENTHS;
  return angle_tenths;
}

// ===== Update Vehicle Position =====
void updatePosition(int speed, int &heading, int angle) {
  if (speed > 0) heading += angle / 10;    // per original sim design
  if (heading >= 3600) heading -= 3600;
  if (heading < 0)     heading += 3600;
  int distance_mm = speed * LOOP_TIME_MS / 1000;
  X_mm += (long)distance_mm * sin1000(heading) / 1000;
  Y_mm += (long)distance_mm * cos1000(heading) / 1000;
}

// ===== Send Wheel Pulse to DBW =====
void sendWheelPulse(int speed_mmPs) {
  if (speed_mmPs <= 0) return;
  unsigned long pulseInterval_ms = (unsigned long)WHEEL_CIRCUM_MM * 1000 / speed_mmPs;
  unsigned long now = millis();
  if (now >= nextPulseTime_ms) {
    digitalWrite(IRPT_WHEEL_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(IRPT_WHEEL_PIN, LOW);
    nextPulseTime_ms = now + pulseInterval_ms;
  }
}

// ===== Send Wheel Angle Sensor Values to DBW =====
void sendAngleSensor(int angleT) {
  int lSense, rSense;
  if (angleT >= 0) {
    lSense = L_STRAIGHT + (L_MIN - L_STRAIGHT) * angleT / MAX_ANGLE_TENTHS;
    rSense = R_STRAIGHT + (R_MIN - R_STRAIGHT) * angleT / MAX_ANGLE_TENTHS;
  } else {
    lSense = L_STRAIGHT + (L_MAX - L_STRAIGHT) * (-angleT) / MAX_ANGLE_TENTHS;
    rSense = R_STRAIGHT + (R_MAX - R_STRAIGHT) * (-angleT) / MAX_ANGLE_TENTHS;
  }
  analogWrite(L_SENSE_PIN, lSense * 4);
  analogWrite(R_SENSE_PIN, rSense * 4);
}
