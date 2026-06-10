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
 * Logs CSV to SD card if available, falls back to Serial.
 * CSV: time_ms, X_mm, Y_mm, heading_tenths, speed_mmPs, angle_tenths, throttle, brakeOn
 *
 * All arithmetic uses integers (no float) per professor's requirement.
 * Angles stored as tenths of a degree. Positions in mm. Speed in mm/s.
 *
 * Reference: https://www.elcanoproject.org/wiki/Communication
 */

#include <SPI.h>
#include <SD.h>

// ===== Pin Definitions =====
// From DBW DAC0
#define THROTTLE_PIN    A0
// From DBW D40
#define BRAKE_VOLT_PIN  42
// From DBW D44
#define BRAKE_ON_PIN    48
// From DBW D26
#define L_TURN_PIN      4
// From DBW D28
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
#define CAN_DRIVE       0x350
// Nav->all: vehicle position east_cm, north_cm
#define CAN_POSITION    0x4C0
// Nav->all: GPS origin lat/lon
#define CAN_ORIGIN      0x251

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
#define MAX_SPEED_mmPs          13600
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
int  computeSpeed(int throttle, bool brakeOn);
int  updateAngle(bool lTurn, bool rTurn);
void updatePosition(int speed, int &heading, int angle);
void sendWheelPulse(int speed);
void sendAngleSensor(int angleT);
void readScriptCommand();
void sendPosition();
void sendOrigin();

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
  Serial.begin(115200);

  // Configure pins
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(BRAKE_VOLT_PIN, INPUT);
  pinMode(BRAKE_ON_PIN, INPUT);
  pinMode(L_TURN_PIN, INPUT);
  pinMode(R_TURN_PIN, INPUT);
  pinMode(IRPT_WHEEL_PIN, OUTPUT);
  digitalWrite(IRPT_WHEEL_PIN, LOW);

  for (int i = 0; i < THROTTLE_HISTORY; i++) throttleHistory[i] = 0;

  // Stage 3: send GPS origin once at startup (binary 0x251 packet)
  sendOrigin();

  // Initialize SD card
  Serial.print("Initializing SD card on pin D");
  Serial.print(SD_CS_PIN);
  Serial.print("... ");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(100);
  if (SD.begin(SD_CS_PIN)) {
    Serial.println("SD card found!");
    char filename[13];
    for (int i = 0; i < 100; i++) {
      sprintf(filename, "SIM%02d.CSV", i);
      if (!SD.exists(filename)) break;
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
      sdAvailable = true;
      Serial.print("Logging to SD: ");
      Serial.println(filename);
    } else {
      Serial.println("Failed to open file. Using Serial.");
    }
  } else {
    Serial.println("SD not found. Using Serial.");
  }

  if (sdAvailable) {
    logFile.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,throttle,brakeOn");
    logFile.flush();
  } else {
    Serial.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,throttle,brakeOn");
  }
  Serial.println("Simulator Stage 1+3 started.");
}

// ===== Main Loop =====
void loop() {
  if (millis() > 50000) {
    if (sdAvailable) {
      logFile.flush();
      logFile.close();
      Serial.println("Done. SD card closed.");
    }
    while (1);
  }

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
    // cm/s -> mm/s
    speed_mmPs   = cmd_speed_cmPs * 10;  
    angle_tenths = cmd_angle_tenths;
    brakeOn      = (cmd_brake > 0);
    throttle     = 0;
    lTurn        = false;
    rTurn        = false;
  } else {
    // Fixed test values (uncomment below to read real DBW inputs)
    throttle = 150;
    brakeOn  = false;
    lTurn    = false;
    rTurn    = false;

    // Uncomment to read real inputs from DBW:
    // int rawThrottle = analogRead(THROTTLE_PIN);
    // throttle = rawThrottle / 4;
    // brakeOn  = digitalRead(BRAKE_ON_PIN);
    // lTurn    = digitalRead(L_TURN_PIN);
    // rTurn    = digitalRead(R_TURN_PIN);

    throttleHistory[historyIndex] = throttle;
    historyIndex = (historyIndex + 1) % THROTTLE_HISTORY;
    speed_mmPs   = computeSpeed(throttle, brakeOn);
    angle_tenths = updateAngle(lTurn, rTurn);
  }

  // Update vehicle position
  updatePosition(speed_mmPs, heading_tenths, angle_tenths);

  // Send simulated sensor values to DBW Arduino
  sendWheelPulse(speed_mmPs);
  sendAngleSensor(angle_tenths);

  // Stage 3: send vehicle position to PC as binary 0x4C0 packet
  sendPosition();

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
    Serial.print(millis());       Serial.print(",");
    Serial.print(X_mm);           Serial.print(",");
    Serial.print(Y_mm);           Serial.print(",");
    Serial.print(heading_tenths); Serial.print(",");
    Serial.print(speed_mmPs);     Serial.print(",");
    Serial.print(angle_tenths);   Serial.print(",");
    Serial.print(throttle);       Serial.print(",");
    Serial.println(brakeOn ? 1 : 0);
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

  while (Serial.available()) {
    char c = (char)Serial.read();
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
        // Debug print so PC can verify Router received the command
        Serial.print("CMD: speed=");   Serial.print(cmd_speed_cmPs);
        Serial.print(" brake=");       Serial.print(cmd_brake);
        Serial.print(" mode=");        Serial.print(cmd_mode);
        Serial.print(" angle=");       Serial.println(cmd_angle_tenths);
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

  // CAN ID high byte
  Serial.write(0x98); 
  // CAN ID low bits | data length
  Serial.write(0x10);  
  // east_cm big-endian int32
  Serial.write((uint8_t)(east_cm >> 24));
  Serial.write((uint8_t)(east_cm >> 16));
  Serial.write((uint8_t)(east_cm >>  8));
  Serial.write((uint8_t)(east_cm      ));
  // north_cm big-endian int32
  Serial.write((uint8_t)(north_cm >> 24));
  Serial.write((uint8_t)(north_cm >> 16));
  Serial.write((uint8_t)(north_cm >>  8));
  Serial.write((uint8_t)(north_cm      ));
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
  Serial.write(0x4A);
  Serial.write(0x30);
  // Byte 1: lat degrees, N hemisphere -> bit 8 = 0
  Serial.write((uint8_t)ORIGIN_LAT);
  // Bytes 2,3,4: lat fraction = 760934
  Serial.write((uint8_t)(ORIGIN_LAT_FRAC >> 16));
  Serial.write((uint8_t)(ORIGIN_LAT_FRAC >>  8));
  Serial.write((uint8_t)(ORIGIN_LAT_FRAC      ));
  // Byte 5: lon degrees = 122
  Serial.write((uint8_t)ORIGIN_LON);
  // Bytes 6,7,8: W -> first bit of byte 6 = 1, fraction = 189963
  uint32_t lon_field = (1UL << 23) | (uint32_t)ORIGIN_LON_FRAC;
  Serial.write((uint8_t)(lon_field >> 16));
  Serial.write((uint8_t)(lon_field >>  8));
  Serial.write((uint8_t)(lon_field      ));
}

// ===== Compute Speed (integer arithmetic) =====
int computeSpeed(int throttle, bool brakeOn) {
  if (brakeOn) { prevSpeed_mmPs = prevSpeed_mmPs * 5000 / 10000; return prevSpeed_mmPs; }
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
  if (speed > 0) heading += angle / 10;
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
