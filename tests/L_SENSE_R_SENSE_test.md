# Wire test: L_SENSE / R_SENSE (Router → DBW analog)

**Result:** Neither wire carries signal. Both DBW inputs read a stable
local board bias, unrelated to what Router writes. ❌

## What was tested

| Wire | Router pin (out) | DBW pin (in) | Signal type |
|---|---|---|---|
| L_SENSE | DAC0 | A10 | analog (0.55 V – 2.75 V → 0–1023 raw) |
| R_SENSE | DAC1 | A11 | analog |

These are the two wires that would carry **simulated wheel-angle
feedback** from Router back to DBW. On the real trike they would carry
voltages from the L and R steering-column potentiometers.

## Why an isolated test was needed

DBW's production sketch was reading `analogRead(A10)` and `analogRead(A11)`
and getting drifting values that didn't match the calibration constants in
`Settings.h`. We *suspected* the wires were open, but we hadn't tested
that in isolation — the production code reads the pins inside a closed PID
loop with CAN + Logger + steering math running, any of which could in
principle perturb the readings. Same kind of inference that turned out to
be wrong for L_TURN/R_TURN, so we ran the same kind of test.

## Method

Two minimal Arduino sketches: `WireTest_SENSE_Router.ino` on Router,
`WireTest_SENSE_DBW.ino` on DBW. **No CAN, no PID, no Logger, no library
beyond Arduino's core.** Just `analogWriteResolution`, `analogWrite`,
`analogRead`, `Serial.print`.

### Router sketch (transmitter)

Sets `analogWriteResolution(12)` so the SAM3X DAC actually receives the
full 0–4095 range (the default is 8-bit, which would clamp values to 255).
Then cycles through 4 phases, 2 seconds each:

| phase | DAC0 | DAC1 |
|---|---|---|
| `LO-HI` | 0 | 4095 |
| `Q1-Q2` | 1365 | 2730 |
| `Q2-Q1` | 2730 | 1365 |
| `HI-LO` | 4095 | 0 |

DAC0 and DAC1 are mirror-image so that if the wires are crossed, we'd see
that in the DBW reads. Four discrete levels per channel let us check
linearity (not just on/off).

Prints CSV: `t_ms, phase, write_DAC0, write_DAC1` at 5 Hz.

### DBW sketch (receiver)

Configures A10 and A11 as `INPUT`. Reads `analogRead(A10)` and
`analogRead(A11)` and prints them at 5 Hz.

Default `analogRead` resolution on Due is 10-bit (0–1023). Left as-is so
the readings are directly comparable to the production `Settings.h`
calibration constants (`Left_Straight_Read = 722`, etc.) which are
expressed in the same 10-bit raw units.

### Expected readings if wires work

Due's DAC output range is approximately 0.55 V to 2.75 V (not 0–3.3 V),
into DBW's 3.3 V analog reference. So a working wire should give:

| Router DAC value | Approx voltage | DBW expected raw |
|---|---|---|
| 0 | 0.55 V | ~170 |
| 1365 | 1.28 V | ~398 |
| 2730 | 2.02 V | ~625 |
| 4095 | 2.75 V | ~853 |

A range of ~683 raw between extremes. If DBW's reads cycle through these
4 levels in the same order Router cycles its DAC, the wires work. If they
stay in a narrow band regardless of Router's output, the wires are open.

## Results

### Router side (transmit)

Router cycled cleanly through all four phases with no anomalies. Phase
boundaries every 2 s, DAC values exactly as commanded.

### DBW side (receive)

Across 25 seconds of capture covering ~3 full cycles, DBW's readings were:

| Channel | Min raw | Max raw | Range |
|---|---|---|---|
| A10 (L_SENSE) | 697 | 733 | **~36** (vs. ~683 expected) |
| A11 (R_SENSE) | 379 | 398 | **~19** (vs. ~683 expected) |

A10 sat tightly around **~720**, A11 around **~390**. Neither correlated
with Router's phase changes — even when Router transitioned from `LO-HI`
(DAC0 = 0) to `HI-LO` (DAC0 = 4095), DBW's A10 reading shifted by less
than 30 raw, well within ADC noise.

Phase-aligned comparison (Router timestamps offset from DBW by ~12.4 s,
matched by cycle pattern):

| Router phase (DAC0 / DAC1) | Expected DBW A10 / A11 | Observed DBW A10 / A11 |
|---|---|---|
| LO-HI (0 / 4095) | ~170 / ~853 | ~722 / ~389 |
| Q1-Q2 (1365 / 2730) | ~398 / ~625 | ~720 / ~387 |
| Q2-Q1 (2730 / 1365) | ~625 / ~398 | ~717 / ~384 |
| HI-LO (4095 / 0) | ~853 / ~170 | ~710 / ~381 |

The observed delta is **~2.5 % of the expected signal range**, on the order
of analog noise, not a working signal.

## Conclusion

The L_SENSE (Router DAC0 → DBW A10) and R_SENSE (Router DAC1 → DBW A11)
bridge wires do not carry signal in either direction in any usable way.
DBW's analog inputs are reading a stable local board bias of approximately
2.32 V on A10 and 1.25 V on A11 — likely from a board-level voltage
divider or undriven-input bias — and Router's DAC outputs are not
reaching them.

## A coincidence to flag

DBW's stable A10 reading of **~720** raw is almost exactly the production
`Settings.h` constant `Left_Straight_Read = 722`. That calibration was
likely measured against this floating-bias value rather than against a
real sensor signal. When real steering pots are eventually connected to
A10 and A11 (real-trike bring-up), `Left_Straight_Read`,
`Right_Straight_Read`, and the corresponding `Read_at_MIN_TURN` /
`Read_at_MAX_TURN` constants will need to be re-measured against actual
pot voltages at known wheel angles.

## Implication for the simulator architecture

DBW's steering PID needs a measurement of the actual wheel angle to know
when to stop driving L_TURN / R_TURN. With L_SENSE / R_SENSE unable to
deliver that measurement on the Bridge, the simulator uses an alternate
feedback channel: **CAN frame `0x430 SimSteerActual`**, published by
Router every loop, carrying its internal `angle_tenths` value. DBW reads
this frame and uses the value in place of `analogRead(A10/A11)`. The PID
logic itself is identical.

On the real trike, this frame is not published; DBW reads the real
potentiometer voltages on A10 and A11 and the rest of the firmware is
unchanged. Same control loop, two interchangeable feedback sources.

## Implication for Folsom

These wires (Router DAC0 → DBW A10, Router DAC1 → DBW A11) appear absent
on the Bridge. The two isolated minimal sketches above produce repeatable
evidence: DBW's reads do not respond to Router's DAC writes at all.
Possible explanations include traces never wired in fab, broken between
fab and now, or a different intended physical mapping than the one
documented in `DBW_Pins.h` and `simulator_stage1.ino`.

## Files

- `WireTest_SENSE_Router/WireTest_SENSE_Router.ino` — Router transmitter sketch
- `WireTest_SENSE_DBW/WireTest_SENSE_DBW.ino` — DBW receiver sketch
