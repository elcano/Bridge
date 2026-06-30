# Proposed assertions for `test_steering_comprehensive.csv`

A proposal — not a code change — showing how `# assert` lines could be
added to the existing `Non_grapic_simulator/test_steering_comprehensive.csv`
so the test produces pass/fail when run through `tools/test_runner.py`.

## Why this file

- Single axis (steering only; speed held at 15 m/s throughout). Easier to
  reason about than the brake or combined tests.
- Already covers half-lock, full-lock, and return to center in both
  directions — the full envelope of the closed-loop steering control.
- Closed-loop behavior is the headline change in the recent
  `closed-loop-steering` / `full-sim-loop` PRs, so this is the natural
  first test to make scorable.

## Expected behavior the assertions should capture

In through-DBW mode, when Nav publishes `0x350` with a target angle,
two things happen:

1. **DBW's steering PID** drives `L_TURN` / `R_TURN` whenever the
   measured angle is outside the deadband (`±5 deg×10`) of the target.
2. **Router** integrates `angle_tenths` by `ANGLE_CHANGE_TENTHS=20` each
   100 ms loop while a direction wire is asserted, until the deadband is
   reached and DBW stops driving.

Slew is therefore ~200 deg×10 / second. End-to-end latency from
"Nav publishes 0x350" → "Router's angle responds" is a few hundred ms
(Sensor_Hub loop period + DBW PID period + CAN propagation). So
assertions should sample **at least ~1 s after the command of interest**
to give the loop time to settle.

The assertions reference `actual_angle_tenths`, which the Sensor_Hub
runner LOG line populates from DBW's `0x400` Actual frame (the value
DBW's PID is closing against).

## Proposed file

(File with `# assert` lines spliced in. Existing content unchanged.)

```csv
# Comprehensive Steering Test
# Format: time_ms, CANID, nbytes, speed_cmPs, brake, mode, angle_tenths
# Description: Test left turn, right turn, and return to straight

# Start: Turn left gradually
0,350,6,1500,0,1,-105
500,350,6,1500,0,1,-210
1000,350,6,1500,0,1,-210
# assert t=1800: -230 <= actual_angle_tenths <= -190

# Return to straight
2000,350,6,1500,0,1,0
2500,350,6,1500,0,1,0
# assert t=3400: -20 <= actual_angle_tenths <= 20

# Turn right gradually
3500,350,6,1500,0,1,105
4000,350,6,1500,0,1,210
4500,350,6,1500,0,1,210
# assert t=5300: 190 <= actual_angle_tenths <= 230

# Return to straight
5500,350,6,1500,0,1,0
6000,350,6,1500,0,1,0
# assert t=6900: -20 <= actual_angle_tenths <= 20
```

## What each assertion checks

| t (ms) | Range | What it verifies |
|--------|-------|------------------|
| 1800 | `[-230, -190]` | After ~1.3 s of commanding full left (-210), the modeled wheel angle has settled within ±20 of the target. ~200 ms slack on top of the theoretical slew time. |
| 3400 | `[-20, +20]` | After ~1.4 s of commanding centered (0), the wheel has returned through the deadband. ±20 covers the steering deadband (5) plus a few-loop transit. |
| 5300 | `[+190, +230]` | Symmetric right-side check of the same convergence behavior. |
| 6900 | `[-20, +20]` | Final return-to-center after the right-side maneuver. Closes the test. |

## Tolerance rationale

- **±20 deg×10 (=2°)** is generous on purpose for a first cut. The
  deadband alone is ±5, so any settled state is within ±5 of the target
  by construction. Padding to ±20 covers per-loop overshoot during the
  PID's last 1-2 cycles plus measurement quantization in the 0x430
  feedback frame. Tighter once the test runs reliably.
- The **800 ms window between final command and assertion** is the
  expected slew time (full-left → full-right is 2.1 s of pure Router
  slew, plus PID/CAN latency) rounded up to give the loop time to
  settle even on a slow Nav-loop tick.

## Failure modes the assertions catch

- **Deadband regression on DBW.** If the steering deadband gets widened
  (e.g. someone bumps `DEADBAND_DegX10` from 5 to 30), the wheel would
  stop short of the target and the ±20 tolerance would catch it.
- **Steering direction inverted** (L/R pins swapped). Commanded -210
  would produce +210, blowing the first assertion.
- **0x350 not arriving at DBW.** Router's wheel never moves; `actual_angle_tenths`
  stays at 0, first assertion fails.
- **0x430 feedback missing.** DBW's open-loop fallback kicks in but
  models the wrong rate; the wheel-vs-CAN values diverge over the test
  and at least one assertion drifts out of tolerance.
- **PID integral runaway.** Wheel overshoots ±230 and the assertion's
  upper bound catches it.

## How to apply

Not directly — propose the asserts as a PR against the file's owner.
The same scorer works against the file with or without the asserts; this
proposal is additive.

If a future PR splices in these asserts, the test becomes the first
through-DBW closed-loop check in the project that produces machine-readable
pass/fail.
