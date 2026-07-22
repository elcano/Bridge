#pragma once

/*
 * Shared physics engine for the Router simulator sketches.
 *
 * Used by:
 *   simulator_stage1/simulator_stage1.ino         (standalone variant)
 *   simulator_closed_loop/simulator_closed_loop.ino (bridge closed-loop variant)
 *
 * Each sketch includes this header with:
 *   #include <simulator_physics.h>
 *
 * --- IDE setup (one-time per developer) ---
 * This is an Arduino library bundled in the repo at
 * Non_grapic_simulator/libraries/simulator_physics/. For the Arduino IDE to
 * find it, set:
 *   File -> Preferences -> Sketchbook location: <repo>/Non_grapic_simulator
 * Then both sketches resolve <simulator_physics.h> via the IDE's standard
 * library search path. Alternatively, symlink (or copy) the library folder
 * into ~/Arduino/libraries/.
 *
 * --- Implementation notes ---
 * Functions are `static inline` so each sketch gets its own translation-unit
 * copy without linker conflicts. They reference macros and globals that the
 * including sketch MUST define BEFORE the include line. Required:
 *
 *   Macros:
 *     ANGLE_CHANGE_TENTHS, MAX_ANGLE_TENTHS
 *     MIN_EFFECTIVE_THROTTLE, MAX_EFFECTIVE_THROTTLE, MAX_SPEED_mmPs
 *     FRICTION_NUM, FRICTION_DEN
 *     THROTTLE_HISTORY, THROTTLE_DELAY_START, THROTTLE_DELAY_END
 *     WHEEL_CIRCUM_MM, LOOP_TIME_MS
 *     L_STRAIGHT, L_MIN, L_MAX, R_STRAIGHT, R_MIN, R_MAX
 *     IRPT_WHEEL_PIN, L_SENSE_PIN, R_SENSE_PIN
 *     BRAKE_DECEL_mmPsPs  (NEW - see brake model note below)
 *
 *   Globals declared in the .ino:
 *     int   throttleHistory[THROTTLE_HISTORY];
 *     int   historyIndex;
 *     int   prevSpeed_mmPs;
 *     int   angle_tenths;
 *     long  X_mm, Y_mm;
 *     unsigned long nextPulseTime_ms;
 *
 * If the two sketches diverge on a constant (e.g. MAX_SPEED_mmPs:
 * standalone=13600, closed-loop=2000), each gets the correct value
 * because macros resolve at each .ino's compilation, not here.
 *
 * Brake model in computeSpeed: UPDATED to a constant-deceleration (linear)
 * model. The previous half-decay-per-loop model (PR #14 by Minhee) stopped
 * the vehicle almost instantly (~0.4s regardless of speed), which Professor
 * Folsom flagged as unrealistically fast. This version decelerates by a
 * fixed BRAKE_DECEL_mmPsPs each loop instead, giving a straight-line speed
 * ramp down to zero. Verified against SD-card CSV log: from 6735 mm/s the
 * speed dropped by exactly 400 mm/s every 100ms loop (with
 * BRAKE_DECEL_mmPsPs=4000, i.e. 4 m/s^2), reaching 0 in ~1.7s.
 * BRAKE_DECEL_mmPsPs is a placeholder value; adjust once real trike braking
 * data is available.
 */

// ===========================================================================
// Integer sine/cosine (returns value * 1000)
// ===========================================================================
static inline int sin1000(int angle_tenths) {
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

static inline int cos1000(int angle_tenths) {
  return sin1000(angle_tenths + 900);
}

// ===========================================================================
// Compute Speed (integer arithmetic).
// Brake model: constant (linear) deceleration while braking, instead of
// half-decay per loop. See file header note above for rationale/history.
// Requires the including .ino to #define BRAKE_DECEL_mmPsPs before this
// header is included (mm/s of speed lost per second while braking).
// ===========================================================================
static inline int computeSpeed(int throttle, bool brakeOn) {
  if (brakeOn) {
    int decel = BRAKE_DECEL_mmPsPs * LOOP_TIME_MS / 1000;
    prevSpeed_mmPs -= decel;
    if (prevSpeed_mmPs < 0) prevSpeed_mmPs = 0;
    return prevSpeed_mmPs;
  }
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

// ===========================================================================
// Update Steering Angle (integrates L_TURN/R_TURN inputs each loop)
// ===========================================================================
static inline int updateAngle(bool lTurn, bool rTurn) {
  if (lTurn && !rTurn)      angle_tenths -= ANGLE_CHANGE_TENTHS;
  else if (!lTurn && rTurn) angle_tenths += ANGLE_CHANGE_TENTHS;
  if (angle_tenths > MAX_ANGLE_TENTHS)  angle_tenths = MAX_ANGLE_TENTHS;
  if (angle_tenths < -MAX_ANGLE_TENTHS) angle_tenths = -MAX_ANGLE_TENTHS;
  return angle_tenths;
}

// ===========================================================================
// Update Vehicle Position (dead-reckon X/Y from speed/heading/angle)
// ===========================================================================
static inline void updatePosition(int speed, int &heading, int angle) {
  if (speed > 0) heading += angle / 10;
  if (heading >= 3600) heading -= 3600;
  if (heading < 0)     heading += 3600;
  int distance_mm = speed * LOOP_TIME_MS / 1000;
  X_mm += (long)distance_mm * sin1000(heading) / 1000;
  Y_mm += (long)distance_mm * cos1000(heading) / 1000;
}

// ===========================================================================
// Send wheel-tick pulse to DBW at a rate proportional to speed
// ===========================================================================
static inline void sendWheelPulse(int speed_mmPs) {
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

// ===========================================================================
// Drive L_SENSE / R_SENSE DACs to simulate wheel-angle pot voltages
// ===========================================================================
static inline void sendAngleSensor(int angleT) {
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
