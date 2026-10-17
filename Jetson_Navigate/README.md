# Jetson_Navigate

Python port of [`Navigate/`](../Navigate) for the Jetson Nano. It replaces the
Sensor Hub Arduino Due as the Navigation node on the Elcano CAN bus, using the
same CAN protocol, waypoint mission, and steering control law.

## System path

```
Jetson Nano (jetson_navigate.py)
   │  USB-CAN adapter (SocketCAN, 500 kbps)
   ▼
Bridge board CAN bus
   ├── DBW Due     – real Drive-by-Wire firmware (steering, throttle, brake)
   └── Router Due  – closed-loop simulator (simulated trike position/heading/speed)
```

In the demo, the simulated trike drove through all four waypoints and stopped.

## What it does (10 Hz loop)

1. Reads pose from CAN and converts waypoints from lat/lon to a flat-earth
   cm frame with waypoint 0 as the origin (port of `NavMath.cpp`).
2. Computes distance and bearing to the active waypoint and a proportional
   steering command from the heading error, capped at ±30°.
3. Advances to the next waypoint inside a 300 cm capture radius; brakes and
   stops after the last one.
4. On Ctrl-C, sends brake-on / speed 0 / e-stop before exiting.

## CAN messages

| Direction | ID | Name | Content |
|---|---|---|---|
| In  | `0x4C0` | VehiclePosition | east_cm, north_cm (int32 LE) |
| In  | `0x4E0` | VehicleHeading  | heading_centiDeg (uint16 LE, 0 = N, +CW) |
| In  | `0x4F0` | VehicleSpeed    | speed_cmPs (int16 LE) |
| In  | `0x400` | Actual          | actual steering angle, DegX10 (from DBW) |
| Out | `0x350` | NavDrive        | speed_cmPs, brake, angle_DegX10 (int16 LE) |
| Out | `0x100` | NavStatus       | 0x40 autonomous, 0x80 e-stop |
| Out | `0x101` | GoalReached     | waypoint index; 0x80 when mission complete |

## Setup and run

Tested on a Jetson Nano (JetPack 4.x, Python 3.6) with a USB-CAN adapter
exposed as SocketCAN.

```bash
sudo ip link set up can0 type can bitrate 500000
pip3 install -r requirements.txt
python3 jetson_navigate.py            # defaults: --channel can0 --bustype socketcan --bitrate 500000
```

Tuning constants (cruise speed, Kp, capture radius, max steering, waypoints)
are at the top of `jetson_navigate.py` and should match `Navigate/Settings.h`
and `Navigate/Waypoints.h`.
