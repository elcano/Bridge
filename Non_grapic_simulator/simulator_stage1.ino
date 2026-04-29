/*
 * Simulator Stage 1
 * Router Arduino Due
 *
 * Reads inputs from DBW Arduino:
 *   A0  -> THROTTLE
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
 * Logs CSV to SD card if available, falls back to Serial.
 * CSV: time_ms, X_mm, Y_mm, heading_tenths, speed_mmPs, angle_tenths, throttle, brakeOn
 *
 * All arithmetic uses integers (no float) per professor's requirement.
 * Angles are stored as tenths of a degree (e.g. 255 = 25.5 deg).
 * Positions are in mm. Speed is in mm/s.
 */

#include <SPI.h>
#include <SD.h>

// ===== Pin Definitions =====
#define THROTTLE_PIN    A0   // From DBW DAC0
#define BRAKE_VOLT_PIN  42   // From DBW D40
#define BRAKE_ON_PIN    48   // From DBW D44
#define L_TURN_PIN      4    // From DBW D26
#define R_TURN_PIN      2    // From DBW D28
#define IRPT_WHEEL_PIN  47   // To DBW D47
#define L_SENSE_PIN     DAC0 // To DBW A10
#define R_SENSE_PIN     DAC1 // To DBW A11

// ===== SD Card Pin =====
// Connect Sensor Hub Pin 35 to 37 (router pin D20)
#define SD_CS_PIN       37

// ===== Speed Model Constants =====
// FRICTION = 0.9296 represented as 9296/10000
#define FRICTION_NUM            9296
#define FRICTION_DEN            10000
#define MIN_EFFECTIVE_THROTTLE  65
#define MAX_EFFECTIVE_THROTTLE  227
#define MAX_SPEED_mmPs          13600  // 13.6 m/s in mm/s
#define THROTTLE_HISTORY        10     // Number of throttle samples to keep
#define THROTTLE_DELAY_START    3      // Start index for mean (from t-10)
#define THROTTLE_DELAY_END      10     // End index for mean (to t-3, 8 samples)

// ===== Vehicle Settings =====
#define WHEEL_DIAMETER_MM   495        // mm (rounded from 495.3)
#define WHEEL_CIRCUM_MM     1555       // mm (495 * pi, rounded)
#define LOOP_TIME_MS        100        // Loop period in ms
#define MAX_ANGLE_TENTHS    250        // 25.0 degrees in tenths
#define ANGLE_CHANGE_TENTHS 20         // 2.0 degrees per loop in tenths

// ===== Wheel Angle Sensor =====
// L_SENSE: Straight=722, Min(hard right)=779, Max(hard left)=639
#define L_STRAIGHT  722
#define L_MIN       779
#define L_MAX       639
// R_SENSE: Straight=731, Min(hard right)=673, Max(hard left)=786
#define R_STRAIGHT  731
#define R_MIN       673
#define R_MAX       786

// ===== Global Variables (all integer) =====
int throttleHistory[THROTTLE_HISTORY];  // Ring buffer of recent throttle values
int historyIndex = 0;

int speed_mmPs     = 0;  // Current speed in mm/s
int prevSpeed_mmPs = 0;  // Previous speed in mm/s

int heading_tenths = 0;  // Vehicle heading in tenths of degree (0=North, 900=East)
int angle_tenths   = 0;  // Steering angle in tenths of degree (negative=left, positive=right)

long X_mm = 0;           // East-West position in mm
long Y_mm = 0;           // North-South position in mm

unsigned long nextPulseTime_ms = 0;  // Timestamp of next wheel pulse

// ===== SD Card Variables =====
File logFile;
bool sdAvailable = false;

// ===== Function Declarations =====
int  computeSpeed(int throttle, bool brakeOn);
int  updateAngle(bool lTurn, bool rTurn);
void updatePosition(int speed, int &heading, int angle);
void sendWheelPulse(int speed);
void sendAngleSensor(int angle_tenths);

// ===== Integer sine/cosine (returns value * 1000) =====
// Uses lookup table for 0-90 degrees, maps other quadrants
// angle_tenths: angle in tenths of degree (0-3600)
int sin1000(int angle_tenths) {
  // Normalize to 0-3599
  angle_tenths = ((angle_tenths % 3600) + 3600) % 3600;
  // sin lookup table for 0-90 deg in 1-deg steps (* 1000)
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

  // Initialize throttle history buffer
  for (int i = 0; i < THROTTLE_HISTORY; i++) {
    throttleHistory[i] = 0;
  }

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
    Serial.print("Opening file: ");
    Serial.println(filename);
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
    Serial.println("Check: JP2 jumpers x3, wire SH Pin35 to TP12(D20)");
  }

  // Print CSV header
  // Units: time=ms, X/Y=mm, heading/angle=tenths of degree, speed=mm/s
  if (sdAvailable) {
    logFile.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,throttle,brakeOn");
    logFile.flush();
  } else {
    Serial.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,throttle,brakeOn");
  }
  Serial.println("Simulator Stage 1 started.");
}

// ===== Main Loop =====
void loop() {
  // Stop after 50 seconds
  if (millis() > 50000) {
    if (sdAvailable) {
      logFile.println("Done.");
      logFile.flush();
      logFile.close();
      Serial.println("Done. SD card closed.");
    } else {
      Serial.println("Done.");
    }
    while (1);
  }

  uint32_t startTime = millis();

  // Fixed test values (uncomment below to read from DBW pins)
  int throttle   = 150;
  bool brakeOn   = false;
  bool brakeVolt = false;
  bool lTurn     = false;
  bool rTurn     = false;

  // Uncomment to read real inputs from DBW:
  // int rawThrottle = analogRead(THROTTLE_PIN);
  // int throttle    = rawThrottle / 4;
  // bool brakeOn    = digitalRead(BRAKE_ON_PIN);
  // bool brakeVolt  = digitalRead(BRAKE_VOLT_PIN);
  // bool lTurn      = digitalRead(L_TURN_PIN);
  // bool rTurn      = digitalRead(R_TURN_PIN);

  // 2. Update throttle history buffer
  throttleHistory[historyIndex] = throttle;
  historyIndex = (historyIndex + 1) % THROTTLE_HISTORY;

  // 3. Compute speed
  speed_mmPs = computeSpeed(throttle, brakeOn);

  // 4. Update steering angle
  angle_tenths = updateAngle(lTurn, rTurn);

  // 5. Update vehicle position
  updatePosition(speed_mmPs, heading_tenths, angle_tenths);

  // 6. Send simulated sensor values to DBW
  sendWheelPulse(speed_mmPs);
  sendAngleSensor(angle_tenths);

  // 7. Log CSV line (all integers)
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

  // 8. Wait to maintain 100ms loop period
  uint32_t elapsed = millis() - startTime;
  if (elapsed < LOOP_TIME_MS)
    delay(LOOP_TIME_MS - elapsed);
}

// ===== Compute Speed (integer arithmetic) =====
int computeSpeed(int throttle, bool brakeOn) {
  if (brakeOn) { prevSpeed_mmPs = 0; return 0; }

  // Compute mean throttle over samples t-10 to t-3 (8 samples)
  long sum = 0;
  for (int i = THROTTLE_DELAY_START; i <= THROTTLE_DELAY_END; i++) {
    int idx = (historyIndex - i + THROTTLE_HISTORY) % THROTTLE_HISTORY;
    sum += throttleHistory[idx];
  }
  int meanThrottle = (int)(sum / 8);

  // Estimated speed from throttle model (mm/s)
  int estimatedSpeed = 0;
  if (meanThrottle > MIN_EFFECTIVE_THROTTLE) {
    estimatedSpeed = (int)((long)MAX_SPEED_mmPs *
                    (meanThrottle - MIN_EFFECTIVE_THROTTLE) /
                    (MAX_EFFECTIVE_THROTTLE - MIN_EFFECTIVE_THROTTLE));
  }
  if (estimatedSpeed < 0) estimatedSpeed = 0;

  // Momentum: prevSpeed * 0.9296 = prevSpeed * 9296 / 10000
  int momentum = (int)((long)prevSpeed_mmPs * FRICTION_NUM / FRICTION_DEN);

  // Final speed
  int newSpeed = (momentum > estimatedSpeed) ? momentum : estimatedSpeed;
  if (newSpeed > MAX_SPEED_mmPs) newSpeed = MAX_SPEED_mmPs;
  prevSpeed_mmPs = newSpeed;
  return newSpeed;
}

// ===== Update Steering Angle (integer, tenths of degree) =====
int updateAngle(bool lTurn, bool rTurn) {
  if (lTurn && !rTurn)      angle_tenths -= ANGLE_CHANGE_TENTHS;
  else if (!lTurn && rTurn) angle_tenths += ANGLE_CHANGE_TENTHS;

  if (angle_tenths > MAX_ANGLE_TENTHS)  angle_tenths = MAX_ANGLE_TENTHS;
  if (angle_tenths < -MAX_ANGLE_TENTHS) angle_tenths = -MAX_ANGLE_TENTHS;
  return angle_tenths;
}

// ===== Update Vehicle Position (integer arithmetic) =====
void updatePosition(int speed, int &heading, int angle) {
  // heading change per loop: angle_tenths * 0.01 degrees
  // = angle_tenths / 100 tenths = angle_tenths / 1000 * 10 tenths
  if (speed > 0) {
    heading += angle_tenths / 100;  // rough heading change in tenths of degree
  }
  if (heading >= 3600) heading -= 3600;
  if (heading < 0)     heading += 3600;

  // Distance in mm per loop: speed_mmPs * 100ms / 1000
  int distance_mm = speed * LOOP_TIME_MS / 1000;

  // Update X, Y using integer sin/cos (* 1000)
  // X += distance * sin(heading) / 1000
  // Y += distance * cos(heading) / 1000
  X_mm += (long)distance_mm * sin1000(heading) / 1000;
  Y_mm += (long)distance_mm * cos1000(heading) / 1000;
}

// ===== Send Wheel Pulse =====
void sendWheelPulse(int speed_mmPs) {
  if (speed_mmPs <= 0) return;

  // Time for one full wheel revolution at current speed (ms)
  unsigned long pulseInterval_ms = (unsigned long)WHEEL_CIRCUM_MM * 1000 / speed_mmPs;

  unsigned long now = millis();
  if (now >= nextPulseTime_ms) {
    digitalWrite(IRPT_WHEEL_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(IRPT_WHEEL_PIN, LOW);
    nextPulseTime_ms = now + pulseInterval_ms;
  }
}

// ===== Send Wheel Angle Sensor Values =====
void sendAngleSensor(int angle_tenths) {
  // angle_tenths range: -250 to +250 (= -25.0 to +25.0 degrees)
  int lSense, rSense;
  if (angle_tenths >= 0) {
    // Right turn
    lSense = L_STRAIGHT + (L_MIN - L_STRAIGHT) * angle_tenths / MAX_ANGLE_TENTHS;
    rSense = R_STRAIGHT + (R_MIN - R_STRAIGHT) * angle_tenths / MAX_ANGLE_TENTHS;
  } else {
    // Left turn
    lSense = L_STRAIGHT + (L_MAX - L_STRAIGHT) * (-angle_tenths) / MAX_ANGLE_TENTHS;
    rSense = R_STRAIGHT + (R_MAX - R_STRAIGHT) * (-angle_tenths) / MAX_ANGLE_TENTHS;
  }
  // DAC output range: 0~4095 (12-bit)
  analogWrite(L_SENSE_PIN, lSense * 4);
  analogWrite(R_SENSE_PIN, rSense * 4);
}
