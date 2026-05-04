/* CanTest - one-shot CAN protocol test sender
 * Upload to the Sensor Hub Due. After a 3-second boot delay (to give you
 * time to open DBW's serial monitor), runs the protocol test script once
 * and then stops. No interactive commands. Re-flash to re-run.
 *
 * Per elcanoproject.org/wiki/Communication:
 *   0x350 NavDrive:  int16[0]=speed_cmPs, int16[1]=brake, int16[2]=angle_DegX10
 *   0x100 NavStatus: byte0 bits  0x80=E-stop, 0x40=autonomous, 0x04=reverse
 */
#include <due_can.h>

#define NavStatus_CANID 0x100
#define NavDrive_CANID  0x350

#define BOOT_DELAY_MS  3000
#define STEP_DELAY_MS  3000

struct TestStep {
  const char* label;
  bool isDrive;       // true = 0x350, false = 0x100
  int16_t speed;
  int16_t brake;
  int16_t angle;
  uint8_t status;
};

static const TestStep script[] = {
  // 0x350 NavDrive field coverage
  { "speed=100 cm/s (forward)",      true,  100,   0,    0, 0 },
  { "brake=100 (brake on)",          true,    0, 100,    0, 0 },
  { "angle=-150 DegX10 (left)",      true,    0,   0, -150, 0 },
  { "angle=+150 DegX10 (right)",     true,    0,   0,  150, 0 },
  { "all-zero drive frame",          true,    0,   0,    0, 0 },
  // 0x100 NavStatus bit coverage
  { "autonomous (0x40)",             false,   0,   0,    0, 0x40 },
  { "manual (0x00)",                 false,   0,   0,    0, 0x00 },
  { "reverse + autonomous (0x44)",   false,   0,   0,    0, 0x44 },
  { "e-stop (0x80)",                 false,   0,   0,    0, 0x80 },
  { "recover to autonomous (0x40)",  false,   0,   0,    0, 0x40 },
};
static const int NUM_STEPS = sizeof(script) / sizeof(script[0]);

static CAN_FRAME outgoing;

static void sendDrive(int16_t speed, int16_t brake, int16_t angle) {
  outgoing.id = NavDrive_CANID;
  outgoing.length = 6;
  outgoing.data.int16[0] = speed;
  outgoing.data.int16[1] = brake;
  outgoing.data.int16[2] = angle;
  bool ok = Can0.sendFrame(outgoing);
  SerialUSB.print(ok ? "TX 0x350 " : "FAIL 0x350 ");
  SerialUSB.print("speed=");   SerialUSB.print(speed);
  SerialUSB.print(" brake=");  SerialUSB.print(brake);
  SerialUSB.print(" angle=");  SerialUSB.println(angle);
}

static void sendStatus(uint8_t byte0) {
  outgoing.id = NavStatus_CANID;
  outgoing.length = 1;
  outgoing.data.uint8[0] = byte0;
  bool ok = Can0.sendFrame(outgoing);
  SerialUSB.print(ok ? "TX 0x100 0x" : "FAIL 0x100 0x");
  if (byte0 < 0x10) SerialUSB.print('0');
  SerialUSB.println(byte0, HEX);
}

static void runStep(int i, const TestStep& s) {
  SerialUSB.println();
  SerialUSB.print("--- STEP ");
  SerialUSB.print(i + 1);
  SerialUSB.print("/");
  SerialUSB.print(NUM_STEPS);
  SerialUSB.print(": ");
  SerialUSB.print(s.label);
  SerialUSB.println(" ---");
  if (s.isDrive) sendDrive(s.speed, s.brake, s.angle);
  else           sendStatus(s.status);
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
  SerialUSB.print("Test script starts in ");
  SerialUSB.print(BOOT_DELAY_MS / 1000);
  SerialUSB.println("s. Open DBW's serial monitor now.");
  delay(BOOT_DELAY_MS);

  SerialUSB.println();
  SerialUSB.println("=== Running protocol test script ===");
  for (int i = 0; i < NUM_STEPS; i++) {
    runStep(i, script[i]);
    delay(STEP_DELAY_MS);
  }
  SerialUSB.println();
  SerialUSB.println("=== Script complete. No more frames will be sent. ===");
}

void loop() {
  // Idle. Re-flash to re-run the script.
}
