/*
 * WireTest_SENSE_DBW.ino — minimal isolated receiver for the L_SENSE /
 * R_SENSE bridge wires from Router (Arduino Due) to DBW (Due).
 *
 *   Router DAC0 → DBW A10  (L_SENSE)
 *   Router DAC1 → DBW A11  (R_SENSE)
 *
 * NO CAN, NO Logger, NO PID. Just analogRead + Serial. Pair with
 * WireTest_SENSE_Router.ino on the Router Due. Capture both serials for
 * one full 8s cycle and compare against the truth table below.
 *
 * Expected reads when the wires are connected (Due DAC range 0.55V–2.75V
 * into DBW 3.3V ADC, 10-bit):
 *
 *   phase LO-HI : Router DAC0=0    DAC1=4095  ->  A10 ~170, A11 ~853
 *   phase Q1-Q2 : Router DAC0=1365 DAC1=2730  ->  A10 ~398, A11 ~625
 *   phase Q2-Q1 : Router DAC0=2730 DAC1=1365  ->  A10 ~625, A11 ~398
 *   phase HI-LO : Router DAC0=4095 DAC1=0     ->  A10 ~853, A11 ~170
 *
 * If A10/A11 readings stay in a narrow band (~200–400 with random drift)
 * regardless of which phase Router is in, the wires are open.
 *
 * If A10 tracks Router's DAC1 (or A11 tracks DAC0), the wires are
 * cross-connected — still working, but mapped wrong.
 */

#define L_SENSE_PIN  A10   // matches DBW_Pins.h
#define R_SENSE_PIN  A11

void setup() {
  SerialUSB.begin(115200);
  uint32_t waitStart = millis();
  while (!SerialUSB && (millis() - waitStart) < 3000);

  // analogRead default resolution on Due is 10-bit (0-1023). Leave as-is so
  // values are comparable with DBW's production Settings.h calibration
  // constants (Left_Straight_Read = 722, etc.).
  pinMode(L_SENSE_PIN, INPUT);
  pinMode(R_SENSE_PIN, INPUT);

  SerialUSB.println("DBW SENSE WireTest started");
  SerialUSB.println("t_ms,read_A10(L_SENSE),read_A11(R_SENSE)");
}

void loop() {
  static uint32_t lastPrint_ms = 0;
  uint32_t now = millis();
  if (now - lastPrint_ms >= 200) {
    lastPrint_ms = now;
    int a10 = analogRead(L_SENSE_PIN);
    int a11 = analogRead(R_SENSE_PIN);
    SerialUSB.print(now); SerialUSB.print(",");
    SerialUSB.print(a10); SerialUSB.print(",");
    SerialUSB.println(a11);
  }
}
