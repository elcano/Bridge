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
 * CSV: time_ms, X_m, Y_m, heading_deg, speed_kmh, angle_deg, throttle, brakeOn
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
// Connect Sensor Hub Pin 35 to TP12 (router pin D20)
#define SD_CS_PIN       20

// ===== Speed Model Constants =====
#define FRICTION                0.9296f
#define MIN_EFFECTIVE_THROTTLE  65
#define MAX_EFFECTIVE_THROTTLE  227
#define MAX_SPEED_mmPs          13600  // 13.6 m/s in mm/s
#define THROTTLE_HISTORY        10     // Number of throttle samples to keep
#define THROTTLE_DELAY_START    3      // Start index for mean (from t-10)
#define THROTTLE_DELAY_END      10     // End index for mean (to t-3, 8 samples)

// ===== Vehicle Settings =====
#define WHEEL_DIAMETER_MM   495.3f
#define WHEEL_CIRCUM_MM     (WHEEL_DIAMETER_MM * 3.14159f)
#define LOOP_TIME_MS        100        // Loop period in ms
#define MAX_ANGLE_DEG       25.0f      // Maximum steering angle in degrees
#define ANGLE_CHANGE_RATE   2.0f       // Steering angle change per loop

// ===== Wheel Angle Sensor =====
// L_SENSE: Straight=722, Min(hard right)=779, Max(hard left)=639
#define L_STRAIGHT  722
#define L_MIN       779
#define L_MAX       639
// R_SENSE: Straight=731, Min(hard right)=673, Max(hard left)=786
#define R_STRAIGHT  731
#define R_MIN       673
#define R_MAX       786

// ===== Global Variables =====
int throttleHistory[THROTTLE_HISTORY];  // Ring buffer of recent throttle values
int historyIndex = 0;

float speed_mmPs = 0;       // Current speed in mm/s
float prevSpeed_mmPs = 0;   // Previous speed in mm/s

float heading_deg = 0;      // Vehicle heading (0=North, 90=East)
float angle_deg = 0;        // Current steering angle (negative=left, positive=right)

float X_mm = 0;             // East-West position in mm
float Y_mm = 0;             // North-South position in mm

unsigned long lastWheelPulse_ms = 0;  // Timestamp of last wheel pulse
unsigned long nextPulseTime_ms = 0;   // Timestamp of next wheel pulse

// ===== SD Card Variables =====
File logFile;
bool sdAvailable = false;

// ===== Function Declarations =====
float computeSpeed(int throttle, bool brakeOn);
float updateAngle(bool lTurn, bool rTurn);
void updatePosition(float speed, float &heading, float angle);
void sendWheelPulse(float speed);
void sendAngleSensor(float angle);

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
  Serial.print("Initializing SD card... ");
  pinMode(SD_CS_PIN, OUTPUT);
  if (SD.begin(SD_CS_PIN)) {
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

  // Print CSV header
  if (sdAvailable) {
    logFile.println("time_ms,X_m,Y_m,heading_deg,speed_kmh,angle_deg,throttle,brakeOn");
    logFile.flush();
  } else {
    Serial.println("time_ms,X_m,Y_m,heading_deg,speed_kmh,angle_deg,throttle,brakeOn");
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
  angle_deg = updateAngle(lTurn, rTurn);

  // 5. Update vehicle position
  updatePosition(speed_mmPs, heading_deg, angle_deg);

  // 6. Send simulated sensor values to DBW
  sendWheelPulse(speed_mmPs);
  sendAngleSensor(angle_deg);

  // 7. Log CSV line to SD card or Serial
  if (sdAvailable) {
    logFile.print(millis());                         logFile.print(",");
    logFile.print(X_mm / 1000.0f, 3);               logFile.print(",");
    logFile.print(Y_mm / 1000.0f, 3);               logFile.print(",");
    logFile.print(heading_deg, 1);                  logFile.print(",");
    logFile.print(speed_mmPs * 3.6f / 1000.0f, 2); logFile.print(",");
    logFile.print(angle_deg, 1);                    logFile.print(",");
    logFile.print(throttle);                        logFile.print(",");
    logFile.println(brakeOn ? 1 : 0);
    logFile.flush();
  } else {
    Serial.print(millis());                         Serial.print(",");
    Serial.print(X_mm / 1000.0f, 3);               Serial.print(",");
    Serial.print(Y_mm / 1000.0f, 3);               Serial.print(",");
    Serial.print(heading_deg, 1);                  Serial.print(",");
    Serial.print(speed_mmPs * 3.6f / 1000.0f, 2); Serial.print(",");
    Serial.print(angle_deg, 1);                    Serial.print(",");
    Serial.print(throttle);                        Serial.print(",");
    Serial.println(brakeOn ? 1 : 0);
  }

  // 8. Wait to maintain 100ms loop period
  uint32_t elapsed = millis() - startTime;
  if (elapsed < LOOP_TIME_MS)
    delay(LOOP_TIME_MS - elapsed);
}

// ===== Compute Speed =====
float computeSpeed(int throttle, bool brakeOn) {
  if (brakeOn) { prevSpeed_mmPs = 0; return 0; }

  long sum = 0;
  for (int i = THROTTLE_DELAY_START; i <= THROTTLE_DELAY_END; i++) {
    int idx = (historyIndex - i + THROTTLE_HISTORY) % THROTTLE_HISTORY;
    sum += throttleHistory[idx];
  }
  float meanThrottle = sum / 8.0f;

  float estimatedSpeed = 0;
  if (meanThrottle > MIN_EFFECTIVE_THROTTLE)
    estimatedSpeed = 13600.0f * (meanThrottle - MIN_EFFECTIVE_THROTTLE)
                     / (MAX_EFFECTIVE_THROTTLE - MIN_EFFECTIVE_THROTTLE);
  estimatedSpeed = max(0.0f, estimatedSpeed);

  float momentum = FRICTION * prevSpeed_mmPs;
  float newSpeed = max(momentum, estimatedSpeed);
  prevSpeed_mmPs = newSpeed;
  return newSpeed;
}

// ===== Update Steering Angle =====
float updateAngle(bool lTurn, bool rTurn) {
  if (lTurn && !rTurn)      angle_deg -= ANGLE_CHANGE_RATE;
  else if (!lTurn && rTurn) angle_deg += ANGLE_CHANGE_RATE;
  angle_deg = constrain(angle_deg, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
  return angle_deg;
}

// ===== Update Vehicle Position =====
void updatePosition(float speed_mmPs, float &heading, float angle) {
  float headingChange = (speed_mmPs > 0) ? angle * 0.01f : 0;
  heading += headingChange;
  if (heading >= 360) heading -= 360;
  if (heading < 0)    heading += 360;

  float distance_mm = speed_mmPs * (LOOP_TIME_MS / 1000.0f);
  float headingRad  = heading * 3.14159f / 180.0f;
  X_mm += distance_mm * sin(headingRad);  // East-West
  Y_mm += distance_mm * cos(headingRad);  // North-South
}

// ===== Send Wheel Pulse =====
void sendWheelPulse(float speed_mmPs) {
  if (speed_mmPs <= 0) return;
  unsigned long pulseInterval_ms = (unsigned long)(WHEEL_CIRCUM_MM / speed_mmPs * 1000.0f);
  unsigned long now = millis();
  if (now >= nextPulseTime_ms) {
    digitalWrite(IRPT_WHEEL_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(IRPT_WHEEL_PIN, LOW);
    nextPulseTime_ms = now + pulseInterval_ms;
  }
}

// ===== Send Wheel Angle Sensor Values =====
void sendAngleSensor(float angle) {
  int lSense, rSense;
  if (angle >= 0) {
    // Right turn
    lSense = (int)(L_STRAIGHT + (L_MIN - L_STRAIGHT) * angle / MAX_ANGLE_DEG);
    rSense = (int)(R_STRAIGHT + (R_MIN - R_STRAIGHT) * angle / MAX_ANGLE_DEG);
  } else {
    // Left turn
    lSense = (int)(L_STRAIGHT + (L_MAX - L_STRAIGHT) * (-angle) / MAX_ANGLE_DEG);
    rSense = (int)(R_STRAIGHT + (R_MAX - R_STRAIGHT) * (-angle) / MAX_ANGLE_DEG);
  }
  // DAC output range: 0~4095 (12-bit)
  analogWrite(L_SENSE_PIN, lSense * 4);
  analogWrite(R_SENSE_PIN, rSense * 4);
}
