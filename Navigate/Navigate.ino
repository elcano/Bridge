/* Sensor_Hub - High-Level Arduino Due firmware (Sensor Hub slot)
 *
 * Stand-in for the Jetson Nano Navigation computer.
 *
 * IN  (from Router via CAN, closed-loop simulation):
 *   0x4C0 VehiclePosition  bytes 0-3 east_cm  int32 LE
 *                           bytes 4-7 north_cm int32 LE
 *   0x4E0 VehicleHeading   bytes 0-1 heading_centiDeg int16 LE (0 = north, +CW)
 *                           bytes 2-3 speed_cmPs       int16 LE
 *
 * OUT (to DBW via CAN):
 *   0x350 NavDrive    int16[0]=speed_cmPs, int16[1]=brake, int16[2]=angle_DegX10
 *   0x100 NavStatus   byte0 bits  0x80=E-stop, 0x40=autonomous, 0x04=reverse
 *   0x101 GoalReached byte0 0x80 set when final waypoint reached
 *
 * Per project convention (see Common.cpp in elcano/HighLevel and the wiki),
 * all in-system location math is in a flat Cartesian (east_cm, north_cm)
 * frame relative to a startup origin. Lat/lon is only used at the simulator
 * boundary; internal state and CAN messages are integer cm.
 */

#include <due_can.h>
#include "Settings.h"
#include "NavMath.h"
#include "Waypoints.h"
#include "SimProtocol.h"

// ---- CAN IDs ----
#define NavStatus_CANID         0x100
#define GoalReached_CANID       0x101
#define DBWStatus_CANID         0x200
#define NavDrive_CANID          0x350
#define Actual_CANID            0x400
// Router/simulator pose feedback to Nav (closed-loop simulation)
#define VehiclePosition_CANID   0x4C0   // bytes 0-3: east_cm int32 LE, bytes 4-7: north_cm int32 LE
#define VehicleHeading_CANID    0x4E0   // bytes 0-1: heading_centiDeg int16 LE (compass only)
#define VehicleSpeed_CANID      0x4F0   // bytes 0-1: speed_cmPs int16 LE (velocity only, separate frame)

// ---- Origin and waypoints in cm-frame (filled at startup) ----
// Origin is set from waypoints[0] at startup; all internal pose math is
// done in (east_cm, north_cm) integer offsets from this origin.
static Origin nav_origin;
static int32_t waypoints_east_cm[NUM_WAYPOINTS];
static int32_t waypoints_north_cm[NUM_WAYPOINTS];

// ---- Navigation state (cm-frame, integer) ----
static int32_t currentEast_cm = 0;
static int32_t currentNorth_cm = 0;
static int32_t currentHeading_centiDeg = 0;   // 0 = north, 9000 = east
static int16_t currentSpeed_cmPs = 0;
static int16_t actualSteerAngle_DegX10 = 0;   // from DBW 0x400: actual wheel angle
static bool gpsValid = false;

static int waypointIdx = 0;
static bool missionComplete = false;

// ---- CAN frames ----
static CAN_FRAME outgoing;

/* -------------------------------------------------------------------------- */
/* Waypoint progression                                                       */
/* -------------------------------------------------------------------------- */
static void updateWaypoint() {
  if (!gpsValid || missionComplete) return;
  int32_t dist = distance_cm(currentEast_cm, currentNorth_cm,
                             waypoints_east_cm[waypointIdx],
                             waypoints_north_cm[waypointIdx]);
  if (dist < WAYPOINT_RADIUS_CM) {
    waypointIdx++;
    if (waypointIdx >= NUM_WAYPOINTS) {
      missionComplete = true;
      // 0x101 GoalReached with mission-complete bit
      outgoing.id = GoalReached_CANID;
      outgoing.length = 1;
      outgoing.data.uint8[0] = 0x80;
      Can0.sendFrame(outgoing);
    } else {
      // Advance notification: byte0 = next waypoint index (low bits)
      outgoing.id = GoalReached_CANID;
      outgoing.length = 1;
      outgoing.data.uint8[0] = (uint8_t)(waypointIdx & 0x7F);
      Can0.sendFrame(outgoing);
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Drive command                                                              */
/* -------------------------------------------------------------------------- */
static void computeAndSendDrive() {
  int16_t speedCmd = 0;
  int16_t brakeCmd = 100;  // default: brake on
  int16_t angleCmd = 0;
  int32_t dist_cm = 0;
  int32_t bearing_cD = 0;
  int32_t err_cD = 0;

  if (gpsValid && !missionComplete) {
    int32_t tgt_e = waypoints_east_cm[waypointIdx];
    int32_t tgt_n = waypoints_north_cm[waypointIdx];
    dist_cm    = distance_cm(currentEast_cm, currentNorth_cm, tgt_e, tgt_n);
    bearing_cD = bearing_centiDeg(currentEast_cm, currentNorth_cm, tgt_e, tgt_n);
    err_cD     = wrap_centiDeg_signed(bearing_cD - currentHeading_centiDeg);
    // steer_DegX10 = Kp * err_deg = (Kp * err_centiDeg) / 100
    int32_t steer = ((int32_t)Kp_STEERING * err_cD) / 100;
    int32_t cap = (int32_t)MAX_STEER_DEG * 10;
    if (steer >  cap) steer =  cap;
    if (steer < -cap) steer = -cap;
    angleCmd = (int16_t)steer;
    speedCmd = CRUISE_CMPS;
    brakeCmd = 0;
  }

  // Per-loop print so user can see what Nav is doing.
  if (gpsValid && !missionComplete) {
    SerialUSB.print("  OUT : waypoint="); SerialUSB.print(waypointIdx);
    SerialUSB.print(" dist_cm=");         SerialUSB.print(dist_cm);
    SerialUSB.print(" bearing=");         SerialUSB.print(bearing_cD / 100.0, 1);
    SerialUSB.print(" heading=");         SerialUSB.print(currentHeading_centiDeg / 100.0, 1);
    SerialUSB.print(" err=");             SerialUSB.print(err_cD / 100.0, 1);
    SerialUSB.print(" -> speed=");        SerialUSB.print(speedCmd);
    SerialUSB.print(" brake=");           SerialUSB.print(brakeCmd);
    SerialUSB.print(" angle=");           SerialUSB.print(angleCmd);
    SerialUSB.print(" actualAngle=");     SerialUSB.println(actualSteerAngle_DegX10);
  } else if (missionComplete) {
    if (currentSpeed_cmPs > 0) {
      SerialUSB.println("  OUT : Destination arrived, stopping -> speed=0 brake=100 angle=0");
    } else {
      SerialUSB.println("  OUT : Stopped -> speed=0 brake=100 angle=0");
    }
  } else {
    SerialUSB.println("  OUT : (no GPS yet) -> speed=0 brake=100 angle=0");
  }

  // 0x350 NavDrive
  outgoing.id = NavDrive_CANID;
  outgoing.length = 6;
  outgoing.data.int16[0] = speedCmd;
  outgoing.data.int16[1] = brakeCmd;
  outgoing.data.int16[2] = angleCmd;
  static uint32_t txOk = 0, txFail = 0;
  bool ok1 = Can0.sendFrame(outgoing);
  if (ok1) txOk++; else txFail++;

  // 0x100 NavStatus - byte0 bits: 0x40 autonomous, 0x80 e-stop, 0x04 reverse
  outgoing.id = NavStatus_CANID;
  outgoing.length = 1;
  uint8_t status = 0x40;  // autonomous intent
  if (missionComplete) status |= 0x80;  // mission-complete -> request stop
  outgoing.data.uint8[0] = status;
  bool ok2 = Can0.sendFrame(outgoing);
  if (ok2) txOk++; else txFail++;

  // Print TX success/fail count every 2 seconds.
  static uint32_t lastTxDebug_ms = 0;
  if (millis() - lastTxDebug_ms > 2000) {
    lastTxDebug_ms = millis();
    SerialUSB.print("# TX txOk="); SerialUSB.print(txOk);
    SerialUSB.print(" txFail="); SerialUSB.println(txFail);
  }
}

/* -------------------------------------------------------------------------- */
/* Arduino entry points                                                       */
/* -------------------------------------------------------------------------- */
void setup() {
  SerialUSB.begin(115200);
  // Bounded wait so the sketch boots even if no PC is attached to Native USB.
  uint32_t waitStart = millis();
  while (!SerialUSB && (millis() - waitStart) < 3000);

  if (Can0.begin(CAN_BPS_500K)) {
    SerialUSB.println("CAN init OK");
  } else {
    SerialUSB.println("CAN init FAILED");
  }
  // FOUR catch-all RX mailboxes. With only 1 mailbox, DBW's high-rate logger
  // frames (0x701-0x70A every 100ms) crowd out Router's slower 0x4C0/0x4E0/0x4F0.
  // Four RX leaves 4 TX mailboxes — enough for our 0x350 + 0x100 broadcasts.
  Can0.watchFor();
  Can0.watchFor();
  Can0.watchFor();
  Can0.watchFor();

  // Initialize the local Euclidean frame using waypoint 0 as the origin,
  // then convert all waypoints to cm-frame for fast integer math at
  // runtime.
  origin_init(nav_origin, waypoints[0].lat_deg, waypoints[0].lon_deg);
  for (int i = 0; i < NUM_WAYPOINTS; i++) {
    latlon_to_cm(nav_origin, waypoints[i].lat_deg, waypoints[i].lon_deg,
                 waypoints_east_cm[i], waypoints_north_cm[i]);
  }

  SerialUSB.print("Waypoints: ");
  SerialUSB.println(NUM_WAYPOINTS);
  SerialUSB.print("Origin: lat=");  SerialUSB.print(nav_origin.lat, 6);
  SerialUSB.print(" lon=");          SerialUSB.println(nav_origin.lon, 6);
  for (int i = 0; i < NUM_WAYPOINTS; i++) {
    SerialUSB.print("  WP"); SerialUSB.print(i);
    SerialUSB.print(": east_cm="); SerialUSB.print(waypoints_east_cm[i]);
    SerialUSB.print(" north_cm="); SerialUSB.println(waypoints_north_cm[i]);
  }
}

void loop() {
  const uint32_t loopPeriod_ms = 1000UL / NAV_LOOP_HZ;
  uint32_t start_ms = millis();

  // Drain incoming CAN frames FIRST so pose is up-to-date before we compute.
  static uint32_t rxAny = 0;
  static uint32_t rx4C0 = 0;
  static uint32_t rx4E0 = 0;
  static uint32_t lastRxId = 0;
  CAN_FRAME incoming;
  while (Can0.available() > 0) {
    Can0.read(incoming);
    rxAny++;
    lastRxId = incoming.id;
    if (incoming.id == VehiclePosition_CANID && incoming.length == 8) {
      currentEast_cm  = incoming.data.int32[0];
      currentNorth_cm = incoming.data.int32[1];
      gpsValid = true;
      rx4C0++;
    }
    else if (incoming.id == VehicleHeading_CANID && incoming.length >= 2) {
      // Read as uint16 to avoid int16 sign-overflow when heading > 327.67°
      // (Router sends heading*100 as 2-byte unsigned, range 0-35999).
      currentHeading_centiDeg = (int32_t)incoming.data.uint16[0];
      rx4E0++;
    }
    else if (incoming.id == VehicleSpeed_CANID && incoming.length >= 2) {
      currentSpeed_cmPs = incoming.data.int16[0];
    }
    else if (incoming.id == Actual_CANID && incoming.length >= 6) {
      // 0x400 Actual from DBW: int16[0]=actual speed, int16[2]=actual steer angle.
      // The angle reflects what DBW's PID measured from Router's DAC feedback,
      // closing the full loop: SensorHub → DBW PID → Router physics → DAC → DBW → here.
      actualSteerAngle_DegX10 = incoming.data.int16[2];
    }
    // DBWStatus (0x200) still ignored.
  }

  // Every 2 seconds, print a CAN RX summary so we can see what's arriving.
  static uint32_t lastRxDebug_ms = 0;
  if (millis() - lastRxDebug_ms > 2000) {
    lastRxDebug_ms = millis();
    SerialUSB.print("# CAN rxAny="); SerialUSB.print(rxAny);
    SerialUSB.print(" rx4C0=");      SerialUSB.print(rx4C0);
    SerialUSB.print(" rx4E0=");      SerialUSB.print(rx4E0);
    SerialUSB.print(" lastId=0x");   SerialUSB.print(lastRxId, HEX);
    SerialUSB.print(" gpsValid=");   SerialUSB.println(gpsValid ? 1 : 0);
  }

  updateWaypoint();
  computeAndSendDrive();

  // Instead of delay(), spin-drain CAN until the loop period elapses. This
  // keeps the RX mailboxes drained even when DBW floods the bus with logger
  // frames, so Router's 0x4C0/0x4E0/0x4F0 don't get crowded out.
  while ((millis() - start_ms) < loopPeriod_ms) {
    while (Can0.available() > 0) {
      Can0.read(incoming);
      rxAny++;
      lastRxId = incoming.id;
      if (incoming.id == VehiclePosition_CANID && incoming.length == 8) {
        currentEast_cm  = incoming.data.int32[0];
        currentNorth_cm = incoming.data.int32[1];
        gpsValid = true;
        rx4C0++;
      } else if (incoming.id == VehicleHeading_CANID && incoming.length >= 2) {
        // Read as uint16 to avoid int16 sign-overflow when heading > 327.67°.
        currentHeading_centiDeg = (int32_t)incoming.data.uint16[0];
        rx4E0++;
      } else if (incoming.id == VehicleSpeed_CANID && incoming.length >= 2) {
        currentSpeed_cmPs = incoming.data.int16[0];
      }
    }
  }
}
