/*
 * WireTest_THROTTLE_DBW.ino — minimal isolated transmitter for the THROTTLE
 * bridge wire from DBW (Arduino Due) to Router (Due).
 *
 *   DBW DAC0 → Router A0  (THROTTLE_PIN)
 *
 * In the production stack DBW's SpeedController writes its commanded
 * throttle to DAC0 via analogWrite(DAC0, currentThrottle), and Router's
 * simulator_stage1 reads it via analogRead(THROTTLE_PIN) and uses the
 * result to drive its simulated speed model.
 *
 * NO CAN, NO PID, NO Logger, NO library beyond Arduino core. Just
 * analogWriteResolution + analogWrite + Serial. Pair with
 * WireTest_THROTTLE_Router.ino on the Router Due.
 *
 * Cycle: 4 phases × 2 seconds each, repeats forever:
 *
 *   phase MIN  : DAC0 = 0      (≈ 0.55 V on the wire)
 *   phase Q1   : DAC0 = 1365   (≈ 1.28 V)
 *   phase Q2   : DAC0 = 2730   (≈ 2.02 V)
 *   phase MAX  : DAC0 = 4095   (≈ 2.75 V)
 *
 * Due DAC output range is approximately 0.55V to 2.75V (not 0–3.3V), so
 * Router's analogRead (0–3.3V → 0–1023, 10-bit) on a working wire should
 * see roughly:
 *
 *   DAC = 0    → ~170 raw
 *   DAC = 1365 → ~398 raw
 *   DAC = 2730 → ~625 raw
 *   DAC = 4095 → ~853 raw
 *
 * If Router sees a stable narrow-band reading unrelated to which phase
 * we're in, the wire is open (same failure mode as L_SENSE / R_SENSE).
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

  SerialUSB.println("DBW THROTTLE WireTest started — 8s cycle (4 phases x 2s)");
  SerialUSB.println("t_ms,phase,write_DAC0");
}

void loop() {
  uint32_t now = millis();
  uint32_t phase = (now / 2000) % 4;
  int dac0;
  const char* name;
  switch (phase) {
    default:
    case 0: dac0 = 0;    name = "MIN"; break;
    case 1: dac0 = 1365; name = "Q1";  break;
    case 2: dac0 = 2730; name = "Q2";  break;
    case 3: dac0 = 4095; name = "MAX"; break;
  }
  analogWrite(DAC0, dac0);

  static uint32_t lastPrint_ms = 0;
  if (now - lastPrint_ms >= 200) {
    lastPrint_ms = now;
    SerialUSB.print(now);  SerialUSB.print(",");
    SerialUSB.print(name); SerialUSB.print(",");
    SerialUSB.println(dac0);
  }
}
