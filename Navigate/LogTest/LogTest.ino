/* LogTest - one-shot CAN-based logging test sender
 *
 * Upload to any Due wired to the CAN bus. Sends a 0x700 LogHeader once,
 * then a loop of fake log entries (0x701, 0x705, 0x706, 0x707, 0x708,
 * 0x709, 0x70A) so the Sensor Hub running Navigate.ino's SD-logging
 * path can be verified end-to-end without needing the real DBW emit
 * side to be ready.
 *
 * Bench setup:
 *   - Sensor Hub Due: runs Navigate.ino with SD card inserted
 *   - This Due: runs LogTest.ino
 *   - CAN H/L wired between them with terminator at each end
 *
 * After running, pull the SD card and inspect LOG##.CSV — should
 * contain rows like:
 *   rx_ms,time_701,...,util_70A
 *   1234,D2040000,,,,640000000000,5F0096FF03,0000,...,0F
 *
 * Byte layouts of 0x701-0x709 are not specified by the wiki yet
 * (a future version will define a bitmap). The values used below are
 * plausible placeholders; the receiver writes raw hex losslessly so
 * the actual byte semantics don't matter for the SD-writing test.
 */
#include <due_can.h>

#define LogHeader_CANID    0x700
#define LogTime_CANID      0x701
#define LogRC_CANID        0x702
#define LogOp_CANID        0x703
#define LogAuto_CANID      0x704
#define LogDesired_CANID   0x705
#define LogThrottle_CANID  0x706
#define LogBrakes_CANID    0x707
#define LogSteer_CANID     0x708
#define LogPosition_CANID  0x709
#define LogFinalize_CANID  0x70A

#define BOOT_DELAY_MS    3000
#define ENTRY_DELAY_MS    200    // ~5 Hz per entry — slow enough to read
#define NUM_ENTRIES        30    // total entries to send

static CAN_FRAME outgoing;

static void sendFrame(uint32_t id, const uint8_t* data, uint8_t len) {
  outgoing.id = id;
  outgoing.length = len;
  for (int i = 0; i < len && i < 8; i++) outgoing.data.uint8[i] = data[i];
  bool ok = Can0.sendFrame(outgoing);
  SerialUSB.print(ok ? "TX 0x" : "FAIL 0x");
  SerialUSB.print(id, HEX);
  SerialUSB.print(" len=");
  SerialUSB.print(len);
  SerialUSB.print(" data=");
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x10) SerialUSB.print("0");
    SerialUSB.print(data[i], HEX);
  }
  SerialUSB.println();
}

// Pack a 16-bit signed value into two little-endian bytes.
static inline void pack_int16(uint8_t* buf, int16_t v) {
  buf[0] = (uint8_t)(v & 0xFF);
  buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

// Pack a 32-bit unsigned value into four little-endian bytes.
static inline void pack_uint32(uint8_t* buf, uint32_t v) {
  buf[0] = (uint8_t)(v & 0xFF);
  buf[1] = (uint8_t)((v >> 8) & 0xFF);
  buf[2] = (uint8_t)((v >> 16) & 0xFF);
  buf[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void sendLogEntry(int n) {
  SerialUSB.println();
  SerialUSB.print("--- Entry ");
  SerialUSB.print(n + 1);
  SerialUSB.print("/");
  SerialUSB.print(NUM_ENTRIES);
  SerialUSB.println(" ---");

  // 0x701 Log Time: int32 ms (best-guess layout — wiki says "ms" but no bytes)
  uint8_t buf[8];
  pack_uint32(buf, millis());
  sendFrame(LogTime_CANID, buf, 4);
  delay(2);

  // Compute commanded values once; reused by 0x704/0x705/0x708 below.
  int16_t speed = 100;
  int16_t brake = 0;
  int16_t angle = (int16_t)(((n % 8) - 4) * 50);  // -200, -150, ..., 150

  // 0x702 Log RC: 4 RC channel pulse widths (int16 microseconds, 8 bytes).
  // Plausible idle values — center stick (1500us), two aux channels.
  int16_t rc_throttle = 1500;
  int16_t rc_steer    = (int16_t)(1500 + angle);   // mirrors commanded steering
  int16_t rc_aux1     = 1000;
  int16_t rc_aux2     = 2000;
  pack_int16(buf + 0, rc_throttle);
  pack_int16(buf + 2, rc_steer);
  pack_int16(buf + 4, rc_aux1);
  pack_int16(buf + 6, rc_aux2);
  sendFrame(LogRC_CANID, buf, 8);
  delay(2);

  // 0x703 Log Op: operator panel — int16 joystick X + int16 joystick Y (4 bytes).
  // Center joystick X; Y varies slightly to simulate operator input.
  int16_t op_x = 0;
  int16_t op_y = (int16_t)((n % 5) * 100);
  pack_int16(buf + 0, op_x);
  pack_int16(buf + 2, op_y);
  sendFrame(LogOp_CANID, buf, 4);
  delay(2);

  // 0x704 Log Auto: speed/brake/angle DBW received from Nav (same shape as 0x705).
  pack_int16(buf + 0, speed);
  pack_int16(buf + 2, brake);
  pack_int16(buf + 4, angle);
  sendFrame(LogAuto_CANID, buf, 6);
  delay(2);

  // 0x705 Log Desired: int16 speed + int16 brake + int16 angle (active commands).
  pack_int16(buf + 0, speed);
  pack_int16(buf + 2, brake);
  pack_int16(buf + 4, angle);
  sendFrame(LogDesired_CANID, buf, 6);
  delay(2);

  // 0x706 Log Throttle: int16 actual_speed + uint8 throttle + uint8 driveMode
  int16_t actual_speed = (int16_t)(95 + (n % 10));   // jitter around 100
  uint8_t throttle_pwm = 150;
  uint8_t drive_mode = 3;                             // AUTO
  pack_int16(buf + 0, actual_speed);
  buf[2] = throttle_pwm;
  buf[3] = drive_mode;
  sendFrame(LogThrottle_CANID, buf, 4);
  delay(2);

  // 0x707 Log Brakes: uint8 brake_on + uint8 brake_voltage
  buf[0] = 0;     // brake_on = false
  buf[1] = 0;     // brake_voltage = 0
  sendFrame(LogBrakes_CANID, buf, 2);
  delay(2);

  // 0x708 Log Steer: int16 actual_angle + int16 R_sensor + int16 L_sensor
  int16_t actual_angle = angle;                       // pretend tracking
  int16_t r_sensor = (int16_t)(2048 + angle * 5);
  int16_t l_sensor = (int16_t)(2048 - angle * 5);
  pack_int16(buf + 0, actual_angle);
  pack_int16(buf + 2, r_sensor);
  pack_int16(buf + 4, l_sensor);
  sendFrame(LogSteer_CANID, buf, 6);
  delay(2);

  // 0x709 Log Position: int32 east_cm + int32 north_cm
  // Pretend the trike is rolling north at ~100 cm/s for n entries.
  int32_t east_cm = 0;
  int32_t north_cm = (int32_t)(n * 20);  // 20 cm per entry
  pack_uint32(buf + 0, (uint32_t)east_cm);
  pack_uint32(buf + 4, (uint32_t)north_cm);
  sendFrame(LogPosition_CANID, buf, 8);
  delay(2);

  // 0x70A Finalize: uint8 utilization (per-cent)
  buf[0] = (uint8_t)(10 + (n % 6));   // 10-15% utilization
  sendFrame(LogFinalize_CANID, buf, 1);
}

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB);

  if (Can0.begin(CAN_BPS_500K)) {
    SerialUSB.println("CAN init success");
  } else {
    SerialUSB.println("CAN init failed");
    while (1);
  }
  SerialUSB.print("Test starts in ");
  SerialUSB.print(BOOT_DELAY_MS / 1000);
  SerialUSB.println("s. Open Sensor Hub's Serial Monitor on the other Due now.");
  delay(BOOT_DELAY_MS);

  SerialUSB.println();
  SerialUSB.println("=== LogTest: emitting fake CAN log frames ===");

  // 0x700 Log Header sent once to indicate session start.
  uint8_t header[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  sendFrame(LogHeader_CANID, header, 8);

  for (int n = 0; n < NUM_ENTRIES; n++) {
    sendLogEntry(n);
    delay(ENTRY_DELAY_MS);
  }

  SerialUSB.println();
  SerialUSB.println("=== Test complete. Pull SD card on Sensor Hub and check LOG##.CSV ===");
}

void loop() {
  // Idle. Re-flash to re-run.
}
