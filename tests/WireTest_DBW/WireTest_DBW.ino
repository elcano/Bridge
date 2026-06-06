/*
 * WireTest_DBW.ino — minimal isolated transmitter for the L_TURN / R_TURN
 * bridge wires from DBW (Arduino Due) to Router (Arduino Due).
 *
 * NO CAN, NO PID, NO Logger, NO Servo. Just pinMode + digitalWrite + Serial.
 * If Router's matching sketch (WireTest_Router) does not see what this
 * sketch writes, the wire (D26 → D4 and/or D28 → D2) is broken.
 *
 * Cycle: 4 phases × 2 seconds each, repeats forever.
 *   phase 00 : D26 LOW,  D28 LOW
 *   phase L0 : D26 HIGH, D28 LOW
 *   phase 0R : D26 LOW,  D28 HIGH
 *   phase LR : D26 HIGH, D28 HIGH   (yes — proves both can be driven)
 *
 * Also reads its own pins back and prints the readback so we can confirm
 * DBW's OUTPUT mode actually took effect.
 */

#define L_TURN_PIN  26   // matches DBW_Pins.h LEFT_TURN_PIN
#define R_TURN_PIN  28   // matches DBW_Pins.h RIGHT_TURN_PIN

void setup() {
  SerialUSB.begin(115200);
  uint32_t waitStart = millis();
  while (!SerialUSB && (millis() - waitStart) < 3000);

  pinMode(L_TURN_PIN, OUTPUT);
  pinMode(R_TURN_PIN, OUTPUT);
  digitalWrite(L_TURN_PIN, LOW);
  digitalWrite(R_TURN_PIN, LOW);

  SerialUSB.println("DBW WireTest started — 8s cycle (4 phases × 2s)");
  SerialUSB.println("t_ms,phase,write_D26,write_D28,readback_D26,readback_D28");
}

void loop() {
  uint32_t now = millis();
  uint32_t phase = (now / 2000) % 4;
  int L, R;
  const char* name;
  switch (phase) {
    default:
    case 0: L = 0; R = 0; name = "00"; break;
    case 1: L = 1; R = 0; name = "L0"; break;
    case 2: L = 0; R = 1; name = "0R"; break;
    case 3: L = 1; R = 1; name = "LR"; break;
  }
  digitalWrite(L_TURN_PIN, L ? HIGH : LOW);
  digitalWrite(R_TURN_PIN, R ? HIGH : LOW);

  static uint32_t lastPrint_ms = 0;
  if (now - lastPrint_ms >= 200) {
    lastPrint_ms = now;
    int rbL = digitalRead(L_TURN_PIN);
    int rbR = digitalRead(R_TURN_PIN);
    SerialUSB.print(now);          SerialUSB.print(",");
    SerialUSB.print(name);         SerialUSB.print(",");
    SerialUSB.print(L);            SerialUSB.print(",");
    SerialUSB.print(R);            SerialUSB.print(",");
    SerialUSB.print(rbL);          SerialUSB.print(",");
    SerialUSB.println(rbR);
  }
}
