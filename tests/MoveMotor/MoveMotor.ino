// Simple move motor — Arduino Motor Shield Channel A.
// D12 = direction, D9 = brake (LOW = run), D3 = PWM speed.

#define DIR_A    12
#define BRAKE_A  9
#define PWM_A    3

void setup() {
  pinMode(DIR_A,   OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  pinMode(PWM_A,   OUTPUT);

  digitalWrite(DIR_A, HIGH);   // direction (LOW = other way)
  digitalWrite(BRAKE_A, LOW);  // release brake
  analogWrite(PWM_A, 150);     // run at speed 0-255
}

void loop() {
}
