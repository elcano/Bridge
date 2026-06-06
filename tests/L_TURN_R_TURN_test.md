# Wire test: L_TURN / R_TURN (DBW → Router digital)

**Result:** Both wires carry signal cleanly. ✅

## What was tested

| Wire | DBW pin | Router pin | Signal type |
|---|---|---|---|
| L_TURN | D26 | D4 | digital (drive HIGH/LOW) |
| R_TURN | D28 | D2 | digital (drive HIGH/LOW) |

These are the two-wire "turn left / turn right" steering command wires from
DBW to Router on the Bridge board.

## Why an isolated test was needed

In production runs (DBW running its full sketch, Router running its full
simulator, Sensor_Hub running Nav), Router was reading `L=1, R=1` constantly
regardless of what DBW asserted. That looked like dead wires — but the
production sketches have CAN traffic, interrupt activity, and other code
that could equally well be the cause. Without isolating the wires from all
that, we couldn't be sure.

## Method

Two minimal Arduino sketches: `WireTest_DBW.ino` on the DBW Due,
`WireTest_Router.ino` on the Router Due. **No CAN, no PID, no Logger, no
Servo, no library beyond Arduino's core.** Just `pinMode`, `digitalWrite`,
`digitalRead`, `Serial.print`.

### DBW sketch

Cycles through 4 phases, 2 seconds each, repeating:

| phase | D26 (L_TURN) | D28 (R_TURN) |
|---|---|---|
| `00` | 0 | 0 |
| `L0` | 1 | 0 |
| `0R` | 0 | 1 |
| `LR` | 1 | 1 |

After each `digitalWrite`, DBW also `digitalRead`s its own pins and prints
the readback. If the readback ever differs from what was written, the
OUTPUT mode is being overridden by something else on DBW — that would
point to a software/firmware bug on the transmit side rather than a wire
problem.

Prints CSV: `t_ms, phase, write_D26, write_D28, readback_D26, readback_D28`
at 5 Hz.

### Router sketch

Configures D4 and D2 as `INPUT`. Explicitly disables the SAM3X internal
pullup on both pins by poking the `PIO_PUDR` register directly, so the
floating-input behavior is deterministic (without this, the default
post-`pinMode(INPUT)` state of the pullup is not guaranteed across resets).

Prints CSV: `t_ms, read_D4, read_D2` at 5 Hz.

Both sketches use their own `millis()` for timestamps — the two Dues don't
share a clock — so phases are matched by cyclic pattern, not by absolute
time.

## Results

### DBW side (transmit)

Every row had `readback_D26 == write_D26` and `readback_D28 == write_D28`,
across all four phases, for the entire capture. DBW's OUTPUT mode was
fully active — pin 26 and pin 28 were physically toggling between 0 V and
3.3 V exactly as commanded.

### Router side (receive)

Router's `digitalRead(D4)` and `digitalRead(D2)` cycled through exactly
the 4 expected (L, R) combinations in the same order DBW was writing them,
just offset in time due to the independent clocks. Every transition matched.

| Router cycle (2s each) | Router reads (L, R) | Matches DBW phase |
|---|---|---|
| segment A | 0, 1 | `0R` |
| segment B | 1, 1 | `LR` |
| segment C | 0, 0 | `00` |
| segment D | 1, 0 | `L0` |
| segment A' | 0, 1 | `0R` (cycle repeats) |

The discriminating phase is `00` — DBW actively pulls both wires LOW.
Router read **0, 0** during every `00` phase. With a broken wire, Router's
input would have floated (read HIGH on this Due). The clean 0, 0 reading
means the wire actively delivered DBW's LOW.

## Conclusion

The L_TURN (D26 → D4) and R_TURN (D28 → D2) bridge wires carry digital
signal reliably. The production-run `L=1, R=1` symptom was **not** a wire
problem.

## What turned out to be the actual cause

The production simulator (`simulator_stage1.ino`) was relying on the
default state of the SAM3X internal pullup after `pinMode(L_TURN_PIN,
INPUT)`. With the pullup enabled (the default behavior in this Arduino SAM
core version), DBW's LOW couldn't fully pull the input below the chip's
logic threshold through the bridge wire, so Router's `digitalRead`
returned HIGH regardless of what DBW wrote.

The isolated test had explicitly disabled the pullup via `PIO_PUDR`. Once
the same explicit disable was added to the production sketch, Router's
reads tracked DBW's writes correctly and the closed loop closed.

## Files

- `WireTest_DBW/WireTest_DBW.ino` — DBW transmitter sketch
- `WireTest_Router/WireTest_Router.ino` — Router receiver sketch
