/*
 * MotorTurn — simple steering-motor turn test (Arduino Motor Shield Channel A).
 * Gently oscillates the motor back and forth at low speed and prints the angle
 * pot (A10) so you can see the pot follow the motor — good for checking the
 * coupling and doing calibration.
 *
 * Channel A pins:
 *   D12 = direction, D9 = brake (LOW = run, HIGH = hold), D3 = PWM speed
 * Pot: wiper -> A10, ends -> 3.3V / GND.
 *
 * ⚠ It does NOT spin continuously — it turns one way for a short burst, stops,
 *   then the other way, so a coupled pot stays inside its ~270° range. Keep the
 *   speed LOW and a hand on the power the first time. If the pot reading pins at
 *   0 or max, the motor is over-rotating it — stop and shorten BURST_MS / lower
 *   SPEED.
 */

#define DIR_A    12
#define BRAKE_A  9
#define PWM_A    3
#define POT_PIN  A10

const int SPEED    = 0;     // PWM 0-255 — keep LOW so it turns slowly
const int BURST_MS = 600;    // how long it drives each direction
const int STOP_MS  = 800;    // pause (held) between direction changes

void setup() {
  SerialUSB.begin(115200);
  uint32_t t = millis();
  while (!SerialUSB && (millis() - t) < 1500);

  pinMode(DIR_A,   OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  pinMode(PWM_A,   OUTPUT);
  digitalWrite(BRAKE_A, HIGH);   // start held
  analogWrite(PWM_A, 0);
  SerialUSB.println("MotorTurn: gentle back-and-forth. Watch A10 track the motor.");
}

// Drive one direction for ms while printing the pot, then stop+hold.
void turn(bool right, int ms) {
  SerialUSB.println(right ? "-> turning RIGHT" : "-> turning LEFT");
  digitalWrite(DIR_A, right ? HIGH : LOW);
  digitalWrite(BRAKE_A, LOW);          // release brake -> motor runs
  analogWrite(PWM_A, SPEED);
  uint32_t start = millis();
  while (millis() - start < (uint32_t)ms) {
    SerialUSB.print("A10 (pot)="); SerialUSB.println(analogRead(POT_PIN));
    delay(50);
  }
  analogWrite(PWM_A, 0);               // stop
  digitalWrite(BRAKE_A, HIGH);         // hold
}

void loop() {
  turn(true,  BURST_MS);
  delay(STOP_MS);
  turn(false, BURST_MS);
  delay(STOP_MS);
}
