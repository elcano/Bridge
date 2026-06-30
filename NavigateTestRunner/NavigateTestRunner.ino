/*
 * NavigateTestRunner
 * Sensor Hub Arduino Due — PC-driven test gateway for the closed-loop sim.
 *
 * Sister sketch to ../Navigate/Navigate.ino. Where Navigate runs the full
 * autonomous waypoint mission, this sketch turns Sensor Hub into a thin
 * gateway between the PC's test runner (tools/test_runner.py) and the CAN
 * bus, so PC-side test CSVs can drive the through-DBW closed loop and
 * score the result.
 *
 *   Inbound  (PC -> SH):  CMD,<id>,<speed>,<brake>,<mode>,<angle>\n
 *   Outbound (SH -> PC):  ACK,<millis>\n
 *                         LOG,<millis>,<key=val>,...\n
 *
 * Flow per loop (~100 ms):
 *   1. Drain CAN RX -> latch current pose + actual angle + router angle
 *   2. Parse any pending CMD,... line from SerialUSB
 *   3. Once a CMD has been latched, publish 0x350 NavDrive + 0x100
 *      NavStatus every loop and emit a LOG line for the scorer
 *   4. Spin-drain CAN until the loop period elapses (keeps the RX
 *      mailboxes fresh even while DBW floods 0x701..0x70A logger frames)
 *
 * To switch between autonomous nav and test gateway, flash the other
 * sketch — there is no in-firmware mode toggle.
 *
 * See tools/test_runner.py for the PC-side runner that drives this and
 * scores the resulting LOG stream.
 */

#include <due_can.h>

// ===== CAN IDs =====
// Per https://www.elcanoproject.org/wiki/Communication
// SH -> DBW : 0x350 NavDrive (speed/brake/angle), 0x100 NavStatus (auto/estop)
// DBW -> SH : 0x400 Actual (actual speed + actual wheel angle)
// Router -> SH : 0x4C0 position, 0x4E0 heading, 0x4F0 speed
// Router -> DBW (snooped by SH for second-witness logging): 0x430 SimSteerActual
#define NavDrive_CANID          0x350
#define NavStatus_CANID         0x100
#define Actual_CANID            0x400
#define VehiclePosition_CANID   0x4C0
#define VehicleHeading_CANID    0x4E0
#define VehicleSpeed_CANID      0x4F0
#define SimSteerActual_CANID    0x430

// ===== Loop period =====
#define LOOP_PERIOD_MS 100

// ===== Pose state captured from CAN =====
static int32_t currentEast_cm           = 0;
static int32_t currentNorth_cm          = 0;
static int16_t currentHeading_centiDeg  = 0;   // 0 = north, 9000 = east
static int16_t currentSpeed_cmPs        = 0;
static int16_t actualSteerAngle_DegX10  = 0;   // from DBW 0x400 bytes 4-5
static int16_t routerAngle_DegX10       = 0;   // from Router 0x430 — second witness

// ===== Latched command from PC =====
// Republished on CAN every loop once the first CMD arrives. brake defaults
// to 100 (engaged) so DBW holds the trike still if a test forgets to send
// brake=0 before commanding speed > 0.
static int16_t  cmd_speed_cmPs   = 0;
static int16_t  cmd_brake        = 100;
static uint8_t  cmd_mode         = 0;
static int16_t  cmd_angle_DegX10 = 0;
static bool     cmdLatched       = false;
static char     cmdBuf[64];
static int      cmdBufIdx        = 0;

// ===== Forward declarations =====
static void drainCanRx();
static void readTestCommand();
static void publishCmdFrames();
static void printTestLog();

/* -------------------------------------------------------------------------- */
void setup() {
  SerialUSB.begin(115200);
  // Native USB enumerates after sketch boot. Bounded wait so the sketch
  // boots even when no host is attached.
  uint32_t waitStart = millis();
  while (!SerialUSB && (millis() - waitStart) < 3000);

  // CAN at 500 kbps (matches DBW + Router). Catch-all RX filter so all
  // frames hit our drain loop.
  Can0.begin(CAN_BPS_500K);
  Can0.watchFor();

  // Boot banner the PC test runner watches for to confirm it's talking
  // to the test gateway and not autonomous Navigate.
  SerialUSB.println("RDY,test_runner_v0");
}

/* -------------------------------------------------------------------------- */
void loop() {
  uint32_t loopStart = millis();

  drainCanRx();
  readTestCommand();

  if (cmdLatched) {
    publishCmdFrames();
    printTestLog();
  }

  // Spin-drain CAN for the rest of the loop period so RX mailboxes stay
  // emptied while DBW's logger frames flood the bus.
  while ((millis() - loopStart) < LOOP_PERIOD_MS) {
    drainCanRx();
  }
}

/* -------------------------------------------------------------------------- */
/* Capture pose + actual-angle frames from CAN                                */
/* -------------------------------------------------------------------------- */
static void drainCanRx() {
  CAN_FRAME incoming;
  while (Can0.available() > 0) {
    Can0.read(incoming);
    switch (incoming.id) {
      case VehiclePosition_CANID:
        if (incoming.length >= 8) {
          currentEast_cm  = incoming.data.int32[0];
          currentNorth_cm = incoming.data.int32[1];
        }
        break;
      case VehicleHeading_CANID:
        if (incoming.length >= 2) {
          currentHeading_centiDeg = incoming.data.int16[0];
        }
        break;
      case VehicleSpeed_CANID:
        if (incoming.length >= 2) {
          currentSpeed_cmPs = incoming.data.int16[0];
        }
        break;
      case Actual_CANID:
        if (incoming.length >= 6) {
          // bytes 4-5: actual steer angle (deg x 10)
          actualSteerAngle_DegX10 = incoming.data.int16[2];
        }
        break;
      case SimSteerActual_CANID:
        if (incoming.length >= 2) {
          // Router's view of the simulated wheel angle. Snooped here so the
          // scorer can compare DBW-reported actual_angle vs Router-reported
          // router_angle and detect disagreement.
          routerAngle_DegX10 = incoming.data.int16[0];
        }
        break;
      default:
        // Ignore everything else (RC, logger frames, etc.)
        break;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Parse CMD,<id>,<speed>,<brake>,<mode>,<angle> lines from PC                */
/* -------------------------------------------------------------------------- */
static void readTestCommand() {
  while (SerialUSB.available() > 0) {
    char c = (char)SerialUSB.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdBufIdx] = '\0';
      int id, speed, brake, mode, angle;
      if (cmdBufIdx > 4 &&
          sscanf(cmdBuf, "CMD,%i,%i,%i,%i,%i",
                 &id, &speed, &brake, &mode, &angle) == 5) {
        cmd_speed_cmPs   = (int16_t)speed;
        cmd_brake        = (int16_t)brake;
        cmd_mode         = (uint8_t)mode;
        cmd_angle_DegX10 = (int16_t)angle;
        cmdLatched       = true;
        SerialUSB.print("ACK,"); SerialUSB.println(millis());
      }
      cmdBufIdx = 0;
    } else if (cmdBufIdx < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdBufIdx++] = c;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Publish the latched CMD as 0x350 NavDrive + 0x100 NavStatus every loop     */
/* -------------------------------------------------------------------------- */
static void publishCmdFrames() {
  CAN_FRAME f;
  f.extended = false;

  // 0x350 NavDrive: int16 speed_cmPs, int16 brake, int16 angle_DegX10
  f.id = NavDrive_CANID;
  f.length = 6;
  f.data.int16[0] = cmd_speed_cmPs;
  f.data.int16[1] = cmd_brake;
  f.data.int16[2] = cmd_angle_DegX10;
  Can0.sendFrame(f);

  // 0x100 NavStatus: uint8 status. Bit 6 (0x40) asserts autonomous mode
  // so DBW arbitrates to AUTO_RC and actually follows the 0x350 commands.
  f.id = NavStatus_CANID;
  f.length = 1;
  f.data.uint8[0] = 0x40;
  Can0.sendFrame(f);
}

/* -------------------------------------------------------------------------- */
/* Emit one machine-parseable LOG line for the PC-side scorer                 */
/* -------------------------------------------------------------------------- */
static void printTestLog() {
  SerialUSB.print("LOG,");                  SerialUSB.print(millis());
  SerialUSB.print(",east_cm=");             SerialUSB.print(currentEast_cm);
  SerialUSB.print(",north_cm=");            SerialUSB.print(currentNorth_cm);
  SerialUSB.print(",heading_centiDeg=");    SerialUSB.print(currentHeading_centiDeg);
  SerialUSB.print(",actual_angle_tenths="); SerialUSB.print(actualSteerAngle_DegX10);
  SerialUSB.print(",router_angle_tenths="); SerialUSB.print(routerAngle_DegX10);
  SerialUSB.print(",cmd_speed_cmPs=");      SerialUSB.print(cmd_speed_cmPs);
  SerialUSB.print(",cmd_angle_tenths=");    SerialUSB.println(cmd_angle_DegX10);
}
