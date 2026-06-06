/*
 * WireTest_SENSE_Router.ino — minimal isolated transmitter for the
 * L_SENSE / R_SENSE bridge wires from Router (Arduino Due) to DBW (Due).
 *
 *   Router DAC0 → DBW A10  (L_SENSE)
 *   Router DAC1 → DBW A11  (R_SENSE)
 *
 * NO CAN, NO Logger, NO physics. Just analogWriteResolution + analogWrite +
 * Serial. If the wires are good, DBW's matching sketch will see analogRead
 * values that track these DAC outputs in 4 distinct bands.
 *
 * Cycle: 4 phases × 2 seconds each, repeats forever. DAC0 and DAC1 are
 * mirror-image so we can distinguish them in the DBW trace (and detect
 * cross-wiring):
 *
 *   phase LO-HI : DAC0 = 0,    DAC1 = 4095
 *   phase Q1-Q2 : DAC0 = 1365, DAC1 = 2730
 *   phase Q2-Q1 : DAC0 = 2730, DAC1 = 1365
 *   phase HI-LO : DAC0 = 4095, DAC1 = 0
 *
 * Due DAC output range is approximately 0.55V to 2.75V (not 0–3.3V), so
 * DBW's analogRead (0–3.3V → 0–1023) on a working wire should see roughly:
 *
 *   DAC = 0    → ~170 raw
 *   DAC = 1365 → ~398 raw
 *   DAC = 2730 → ~625 raw
 *   DAC = 4095 → ~853 raw
 *
 * If DBW sees floating-noise readings (random drift, not tracking these
 * levels), the wire is open.
 */

void setup() {
  SerialUSB.begin(115200);
  uint32_t waitStart = millis();
  while (!SerialUSB && (millis() - waitStart) < 3000);

  // Due DAC is 12-bit. Default analogWrite resolution is 8-bit, which
  // would clamp our values to 255 max. Set 12-bit so the DAC actually
  // receives 0–4095.
  analogWriteResolution(12);

  analogWrite(DAC0, 2048);  // mid-range while booting
  analogWrite(DAC1, 2048);

  SerialUSB.println("Router SENSE WireTest started — 8s cycle (4 phases x 2s)");
  SerialUSB.println("t_ms,phase,write_DAC0,write_DAC1");
}

void loop() {
  uint32_t now = millis();
  uint32_t phase = (now / 2000) % 4;
  int dac0, dac1;
  const char* name;
  switch (phase) {
    default:
    case 0: dac0 = 0;    dac1 = 4095; name = "LO-HI"; break;
    case 1: dac0 = 1365; dac1 = 2730; name = "Q1-Q2"; break;
    case 2: dac0 = 2730; dac1 = 1365; name = "Q2-Q1"; break;
    case 3: dac0 = 4095; dac1 = 0;    name = "HI-LO"; break;
  }
  analogWrite(DAC0, dac0);
  analogWrite(DAC1, dac1);

  static uint32_t lastPrint_ms = 0;
  if (now - lastPrint_ms >= 200) {
    lastPrint_ms = now;
    SerialUSB.print(now);  SerialUSB.print(",");
    SerialUSB.print(name); SerialUSB.print(",");
    SerialUSB.print(dac0); SerialUSB.print(",");
    SerialUSB.println(dac1);
  }
}
