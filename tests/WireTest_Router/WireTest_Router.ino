/*
 * WireTest_Router.ino — minimal isolated receiver for the L_TURN / R_TURN
 * bridge wires from DBW (Arduino Due) to Router (Arduino Due).
 *
 * NO CAN, NO Logger, NO physics. Just pinMode + digitalRead + Serial.
 *
 * Pins are configured as plain INPUT. Empirically, floating inputs on this
 * Due read HIGH (we've been seeing this all along). So:
 *   Wire works → Router's reads track DBW's writes (varies per phase)
 *   Wire open  → Router stays at L=1 R=1 forever (floats HIGH)
 *
 * Pair with WireTest_DBW.ino on the DBW Due. Capture both serials for a
 * full 8s cycle and compare against the truth table in the README.
 */

#define L_TURN_PIN  4    // matches Router's L_TURN_PIN
#define R_TURN_PIN  2    // matches Router's R_TURN_PIN

void setup() {
  SerialUSB.begin(115200);
  uint32_t waitStart = millis();
  while (!SerialUSB && (millis() - waitStart) < 3000);

  pinMode(L_TURN_PIN, INPUT);
  pinMode(R_TURN_PIN, INPUT);
  // Explicitly disable any pullup so the float behavior is deterministic.
  g_APinDescription[L_TURN_PIN].pPort->PIO_PUDR = g_APinDescription[L_TURN_PIN].ulPin;
  g_APinDescription[R_TURN_PIN].pPort->PIO_PUDR = g_APinDescription[R_TURN_PIN].ulPin;

  SerialUSB.println("Router WireTest started — pulldown enabled on D4, D2");
  SerialUSB.println("t_ms,read_D4(L),read_D2(R)");
}

void loop() {
  static uint32_t lastPrint_ms = 0;
  uint32_t now = millis();
  if (now - lastPrint_ms >= 200) {
    lastPrint_ms = now;
    int L = digitalRead(L_TURN_PIN);
    int R = digitalRead(R_TURN_PIN);
    SerialUSB.print(now); SerialUSB.print(",");
    SerialUSB.print(L);   SerialUSB.print(",");
    SerialUSB.println(R);
  }
}
