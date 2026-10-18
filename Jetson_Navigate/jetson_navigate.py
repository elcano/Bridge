#!/usr/bin/env python3
"""
Jetson-side port of Navigate/Navigate.ino for the Elcano trike.

Runs on a Jetson Nano (or any Linux box) with a USB-CAN adapter exposed as a
SocketCAN interface. Drop-in replacement for the Sensor Hub Arduino Due as
the Nav node on the CAN bus — same wire protocol, same waypoint mission,
same control law.

Python 3.6 compatible (matches Jetson Nano's JetPack 4.x default).

Wire I/O
========
IN  (from Router via CAN, closed-loop simulation):
    0x4C0 VehiclePosition  bytes 0-3 east_cm  int32 LE
                            bytes 4-7 north_cm int32 LE
    0x4E0 VehicleHeading   bytes 0-1 heading_centiDeg uint16 LE (0=N, +CW)
    0x4F0 VehicleSpeed     bytes 0-1 speed_cmPs int16 LE
    0x400 Actual           bytes 4-5 actual_steer_DegX10 int16 LE (from DBW)

OUT (to DBW via CAN):
    0x350 NavDrive    int16[0]=speed_cmPs, int16[1]=brake, int16[2]=angle_DegX10
    0x100 NavStatus   byte0 bits 0x80=E-stop, 0x40=autonomous, 0x04=reverse
    0x101 GoalReached byte0 0x80 set when final waypoint reached

Setup on Jetson
===============
1. Plug USB-CAN adapter (e.g. CANable v2.5 or Innomaker USB2CAN) into a USB port.
2. Bring the interface up at the project bitrate:
       sudo ip link set up can0 type can bitrate 500000
3. Install python-can (one of):
       sudo apt install python3-pip && pip3 install python-can
       sudo apt install python3-can
4. Run:
       python3 jetson_navigate.py
   Common options:
       --channel can0      (default)
       --bustype socketcan (default)
       --bitrate 500000    (default)

Stop the script with Ctrl-C — it sends a brake-on/speed-0/e-stop frame
to DBW before exiting so the trike doesn't keep rolling.

Tuning knobs are at the top of this file (cruise speed, Kp, waypoint
radius, max steering, loop rate, waypoints). Match Sensor Hub's Settings.h
and Waypoints.h.
"""

import argparse
import math
import signal
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.stderr.write("ERROR: python-can not installed. Run:\n")
    sys.stderr.write("    sudo apt install python3-pip && pip3 install python-can\n")
    sys.stderr.write("or\n")
    sys.stderr.write("    sudo apt install python3-can\n")
    sys.exit(2)


# =============================================================================
# Tunables (must match Sensor Hub's Settings.h)
# =============================================================================

CRUISE_CMPS = 100                # cruise speed in cm/s
KP_STEERING = 5                  # DegX10 of steering per degree of heading error
WAYPOINT_RADIUS_CM = 300         # waypoint capture radius in cm
MAX_STEER_DEG = 30               # max steering magnitude in whole degrees
NAV_LOOP_HZ = 10                 # main loop rate
CAN_BITRATE = 500000             # 500 kbps; matches DBW + Router


# =============================================================================
# Waypoints (match Waypoints.h — UW Bothell area, ~20 m square loop)
# =============================================================================

WAYPOINTS = [
    # (lat_deg, lon_deg)
    (47.7600, -122.1917),   # WP0 — origin
    (47.7602, -122.1917),   # WP1 — ~22 m north
    (47.7602, -122.1914),   # WP2 — ~22 m east of WP1
    (47.7600, -122.1914),   # WP3 — ~22 m south
]


# =============================================================================
# CAN protocol constants
# =============================================================================

NAV_STATUS_CANID         = 0x100
GOAL_REACHED_CANID       = 0x101
DBW_STATUS_CANID         = 0x200
NAV_DRIVE_CANID          = 0x350
ACTUAL_CANID             = 0x400
VEHICLE_POSITION_CANID   = 0x4C0
VEHICLE_HEADING_CANID    = 0x4E0
VEHICLE_SPEED_CANID      = 0x4F0


# =============================================================================
# Flat-earth navigation math (port of NavMath.cpp)
# =============================================================================

EARTH_RADIUS_CM = 637100000
TO_RADIANS = math.pi / 180.0


class Origin(object):
    """GPS origin in degrees, with pre-computed cos(lat) for fast lat/lon->cm."""

    def __init__(self, lat_deg, lon_deg):
        self.lat = lat_deg
        self.lon = lon_deg
        self.cos_lat = math.cos(lat_deg * TO_RADIANS)


def latlon_to_cm(origin, lat_deg, lon_deg):
    """Convert lat/lon (degrees) to local (east_cm, north_cm) ints."""
    d_lat = lat_deg - origin.lat
    d_lon = lon_deg - origin.lon
    north_cm = int(d_lat * TO_RADIANS * EARTH_RADIUS_CM)
    east_cm  = int(d_lon * TO_RADIANS * EARTH_RADIUS_CM * origin.cos_lat)
    return east_cm, north_cm


def distance_cm(e1, n1, e2, n2):
    """Euclidean distance between two cm-frame points (returns int cm)."""
    de = e1 - e2
    dn = n1 - n2
    return int(math.sqrt(de * de + dn * dn))


def bearing_centiDeg(e1, n1, e2, n2):
    """Bearing from (e1,n1) to (e2,n2) in centidegrees, range [0, 35999].

    0 = north, 9000 = east, 18000 = south, 27000 = west.
    """
    de = float(e2 - e1)
    dn = float(n2 - n1)
    # atan2(east, north): 0 = pointing north, positive clockwise.
    rad = math.atan2(de, dn)
    deg = rad / TO_RADIANS
    if deg < 0:
        deg += 360.0
    return int(deg * 100.0)


def wrap_centiDeg_signed(x):
    """Wrap a signed centidegree value into (-18000, +18000]."""
    while x > 18000:
        x -= 36000
    while x <= -18000:
        x += 36000
    return x


# =============================================================================
# Navigation state
# =============================================================================

class NavState(object):
    """Mutable nav state — updated from CAN RX, read by the control law."""

    def __init__(self):
        self.east_cm = 0
        self.north_cm = 0
        self.heading_centiDeg = 0     # 0 = N, +CW, range [0, 35999]
        self.speed_cmPs = 0
        self.actual_steer_DegX10 = 0
        self.gps_valid = False
        self.waypoint_idx = 0
        self.mission_complete = False


# =============================================================================
# CAN RX — drain any pending frames into state
# =============================================================================

def drain_can(bus, state, counters):
    """Read every pending frame and latch into NavState. Non-blocking."""
    while True:
        msg = bus.recv(timeout=0.0)
        if msg is None:
            return
        counters["rx_any"] += 1
        counters["last_id"] = msg.arbitration_id
        data = msg.data
        cid = msg.arbitration_id

        if cid == VEHICLE_POSITION_CANID and len(data) >= 8:
            east_cm, north_cm = struct.unpack_from("<ii", data, 0)
            state.east_cm = east_cm
            state.north_cm = north_cm
            state.gps_valid = True
            counters["rx_4c0"] += 1

        elif cid == VEHICLE_HEADING_CANID and len(data) >= 2:
            # Heading is sent as uint16 centideg (range 0-35999), not int16.
            state.heading_centiDeg = struct.unpack_from("<H", data, 0)[0]
            counters["rx_4e0"] += 1

        elif cid == VEHICLE_SPEED_CANID and len(data) >= 2:
            state.speed_cmPs = struct.unpack_from("<h", data, 0)[0]

        elif cid == ACTUAL_CANID and len(data) >= 6:
            # 0x400 Actual from DBW: int16[0]=speed, int16[2]=actual steer angle.
            state.actual_steer_DegX10 = struct.unpack_from("<h", data, 4)[0]


# =============================================================================
# Waypoint progression — advance to next WP when current one is within radius
# =============================================================================

def update_waypoint(bus, state, wp_cm):
    if not state.gps_valid or state.mission_complete:
        return
    tgt_e, tgt_n = wp_cm[state.waypoint_idx]
    if distance_cm(state.east_cm, state.north_cm, tgt_e, tgt_n) >= WAYPOINT_RADIUS_CM:
        return

    state.waypoint_idx += 1
    if state.waypoint_idx >= len(wp_cm):
        state.mission_complete = True
        _try_send(bus, GOAL_REACHED_CANID, bytes([0x80]))
    else:
        _try_send(bus, GOAL_REACHED_CANID, bytes([state.waypoint_idx & 0x7F]))


# =============================================================================
# Drive command — compute steering from bearing-vs-heading error, publish CAN
# =============================================================================

def compute_and_send_drive(bus, state, wp_cm, tx_counters):
    speed_cmd = 0
    brake_cmd = 100   # default: brake on
    angle_cmd = 0
    dist_cm = 0
    bearing_cD = 0
    err_cD = 0

    if state.gps_valid and not state.mission_complete:
        tgt_e, tgt_n = wp_cm[state.waypoint_idx]
        dist_cm   = distance_cm(state.east_cm, state.north_cm, tgt_e, tgt_n)
        bearing_cD = bearing_centiDeg(state.east_cm, state.north_cm, tgt_e, tgt_n)
        err_cD    = wrap_centiDeg_signed(bearing_cD - state.heading_centiDeg)
        # steer_DegX10 = Kp * err_deg = (Kp * err_centiDeg) / 100
        steer = (KP_STEERING * err_cD) // 100
        cap = MAX_STEER_DEG * 10
        if steer > cap:
            steer = cap
        if steer < -cap:
            steer = -cap
        angle_cmd = steer
        speed_cmd = CRUISE_CMPS
        brake_cmd = 0

    # Per-loop human-readable status line (same format as Navigate.ino).
    if state.gps_valid and not state.mission_complete:
        print("  OUT : waypoint={0} dist_cm={1} bearing={2:.1f} heading={3:.1f} "
              "err={4:.1f} -> speed={5} brake={6} angle={7} actualAngle={8}".format(
                  state.waypoint_idx, dist_cm, bearing_cD / 100.0,
                  state.heading_centiDeg / 100.0, err_cD / 100.0,
                  speed_cmd, brake_cmd, angle_cmd, state.actual_steer_DegX10))
    elif state.mission_complete:
        if state.speed_cmPs > 0:
            print("  OUT : Destination arrived, stopping -> speed=0 brake=100 angle=0")
        else:
            print("  OUT : Stopped -> speed=0 brake=100 angle=0")
    else:
        print("  OUT : (no GPS yet) -> speed=0 brake=100 angle=0")

    # 0x350 NavDrive — int16 LE: speed_cmPs, brake, angle_DegX10 (6 bytes)
    drive_data = struct.pack("<hhh", speed_cmd, brake_cmd, angle_cmd)
    if _try_send(bus, NAV_DRIVE_CANID, drive_data):
        tx_counters["tx_ok"] += 1
    else:
        tx_counters["tx_fail"] += 1

    # 0x100 NavStatus — 1 byte: 0x40 autonomous, 0x80 e-stop, 0x04 reverse
    status = 0x40   # autonomous intent
    if state.mission_complete:
        status |= 0x80   # mission complete -> request stop
    if _try_send(bus, NAV_STATUS_CANID, bytes([status])):
        tx_counters["tx_ok"] += 1
    else:
        tx_counters["tx_fail"] += 1


def _try_send(bus, can_id, data):
    try:
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False),
                 timeout=0.05)
        return True
    except can.CanError:
        return False


# =============================================================================
# Clean-exit handler: send brake-on / e-stop before quitting
# =============================================================================

def send_stop(bus):
    # 0x350: speed=0, brake=100, angle=0
    _try_send(bus, NAV_DRIVE_CANID, struct.pack("<hhh", 0, 100, 0))
    # 0x100: e-stop bit set
    _try_send(bus, NAV_STATUS_CANID, bytes([0x80]))


# =============================================================================
# Main
# =============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="Jetson-side Navigate node for the Elcano trike CAN bus.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--bustype", default="socketcan",
                    help="python-can bus type (default: socketcan)")
    ap.add_argument("--channel", default="can0",
                    help="CAN channel name (default: can0)")
    ap.add_argument("--bitrate", type=int, default=CAN_BITRATE,
                    help="CAN bitrate in bps (default: {0})".format(CAN_BITRATE))
    args = ap.parse_args()

    print("Opening CAN bus: bustype={0} channel={1} bitrate={2}".format(
        args.bustype, args.channel, args.bitrate))
    try:
        # Use the `bustype=` kwarg — supported on python-can 3.x (Python 3.6
        # default install) and aliased on 4.x.
        bus = can.interface.Bus(
            bustype=args.bustype,
            channel=args.channel,
            bitrate=args.bitrate,
        )
    except Exception as e:
        sys.stderr.write("ERROR opening CAN bus: {0}\n".format(e))
        sys.stderr.write("Hint: confirm the interface is up. For SocketCAN:\n")
        sys.stderr.write("    sudo ip link set up can0 type can bitrate 500000\n")
        return 1
    print("CAN init OK")

    # ----- Initialize the local Euclidean frame from waypoint 0 -----
    origin_lat, origin_lon = WAYPOINTS[0]
    nav_origin = Origin(origin_lat, origin_lon)
    waypoints_cm = []
    for lat, lon in WAYPOINTS:
        waypoints_cm.append(latlon_to_cm(nav_origin, lat, lon))

    print("Waypoints: {0}".format(len(WAYPOINTS)))
    print("Origin: lat={0:.6f} lon={1:.6f}".format(nav_origin.lat, nav_origin.lon))
    for i, (e, n) in enumerate(waypoints_cm):
        print("  WP{0}: east_cm={1} north_cm={2}".format(i, e, n))

    # ----- State + counters -----
    state = NavState()
    rx_counters = {"rx_any": 0, "rx_4c0": 0, "rx_4e0": 0, "last_id": 0}
    tx_counters = {"tx_ok": 0, "tx_fail": 0}

    # ----- Signal handler: send stop on Ctrl-C / SIGTERM -----
    def shutdown(signum=None, frame=None):
        print("\n# shutdown requested - sending stop frame to DBW")
        try:
            send_stop(bus)
        except Exception:
            pass
        try:
            bus.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ----- Main loop -----
    loop_period_s = 1.0 / NAV_LOOP_HZ
    last_rx_dbg = time.monotonic()
    last_tx_dbg = time.monotonic()

    while True:
        loop_start = time.monotonic()

        # Drain incoming CAN so pose is up-to-date before we compute.
        drain_can(bus, state, rx_counters)

        # Periodic RX summary every 2 seconds.
        now = time.monotonic()
        if now - last_rx_dbg > 2.0:
            last_rx_dbg = now
            print("# CAN rxAny={0} rx4C0={1} rx4E0={2} lastId=0x{3:X} gpsValid={4}".format(
                rx_counters["rx_any"], rx_counters["rx_4c0"],
                rx_counters["rx_4e0"], rx_counters["last_id"],
                1 if state.gps_valid else 0))

        update_waypoint(bus, state, waypoints_cm)
        compute_and_send_drive(bus, state, waypoints_cm, tx_counters)

        # Periodic TX summary every 2 seconds.
        if now - last_tx_dbg > 2.0:
            last_tx_dbg = now
            print("# TX txOk={0} txFail={1}".format(
                tx_counters["tx_ok"], tx_counters["tx_fail"]))

        # Spin-drain CAN until the loop period elapses — keeps RX mailboxes
        # drained even when DBW floods 0x701-0x70A logger frames.
        while time.monotonic() - loop_start < loop_period_s:
            drain_can(bus, state, rx_counters)
            # Yield briefly so we don't peg a CPU core spinning.
            time.sleep(0.001)


if __name__ == "__main__":
    sys.exit(main())
