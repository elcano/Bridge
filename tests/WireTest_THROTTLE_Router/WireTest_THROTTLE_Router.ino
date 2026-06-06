/*
 * WireTest_THROTTLE_Router.ino — minimal isolated receiver for the THROTTLE
 * bridge wire from DBW (Arduino Due) to Router (Due).
 *
 *   DBW DAC0 → Router A0  (THROTTLE_PIN)
 *
 * NO CAN, NO physics, NO library beyond Arduino core. Just analogRead +
 * Serial. Pair with WireTest_THROTTLE_DBW.ino on the DBW Due. Capture both
 * serials for one full 8s cycle and compare against the truth table below.
 *
 * Expected reads when the wire is connected (Due DAC range 0.55V–2.75V
 * into Router 3.3V ADC, 10-bit):
 *
 *   phase MIN  : DBW DAC0=0     ->  A0 ~170 raw
 *   phase Q1   : DBW DAC0=1365  ->  A0 ~398 raw
 *   phase Q2   : DBW DAC0=2730  ->  A0 ~625 raw
 *   phase MAX  : DBW DAC0=4095  ->  A0 ~853 raw
 *
 * If A0's reading stays in a narrow band (e.g. ~160–200) regardless of
 * which phase DBW is in, the wire is open — same failure mode as the
 * L_SENSE / R_SENSE wires documented in L_SENSE_R_SENSE_test.md.
 *
 * Note: this sketch does NOT touch the PIO_PUDR register on A0. The
 * SAM3X analog peripheral normally takes the pin away from the PIO
 * controller on first analogRead, so the PIO pullup-disable trick used
 * for L_TURN / R_TURN should not be required here. If the wire turns
 * out to be alive but the production read is wrong, that's the next
 * thing to test.
 */

#define THROTTLE_PIN  A0   // matches simulator_stage1.ino

void setup() {
  SerialUSB.begin(115200);
  uint32_t waitStart = millis();
  while (!SerialUSB && (millis() - waitStart) < 3000);

  // analogRead default resolution on Due is 10-bit (0-1023). Leave as-is
  // so values are directly comparable with what simulator_stage1.ino sees.
  pinMode(THROTTLE_PIN, INPUT);

  SerialUSB.println("Router THROTTLE WireTest started");
  SerialUSB.println("t_ms,read_A0(THROTTLE)");
}

void loop() {
  static uint32_t lastPrint_ms = 0;
  uint32_t now = millis();
  if (now - lastPrint_ms >= 200) {
    lastPrint_ms = now;
    int a0 = analogRead(THROTTLE_PIN);
    SerialUSB.print(now); SerialUSB.print(",");
    SerialUSB.println(a0);
  }
}
