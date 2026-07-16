# Drive-By-Wire — Corner-Case Test Report

_Elcano autonomous trike · DBW firmware + 2-board hardware-in-the-loop rig_
_Generated: 2026-07-12_

> Scope: this report covers the steering / throttle / brake control path on the
> DBW board, exercised through the Router simulator over CAN, driven by the
> Python test harness (`tools/test_runner.py`, `tools/run_all_tests.py`).

---

## 1. Executive summary — how the trike is looking

| Area | Status | Notes |
|------|--------|-------|
| Steering slew & tracking | 🟢 Healthy | Slews at spec (~20 tenths/100ms), tracks commands, clamps at full lock. |
| Steering clamp / over-limit | 🟢 Healthy | Over-limit commands clamp to full lock, no overshoot/wrap. |
| Throttle / cruise | 🟢 Healthy | Accelerates and holds cruise; realistic throttle-lag ramp. |
| Braking (via speed 0) | 🟢 Healthy | Half-decay stop works; all speed-0 brake tests pass. |
| **Brake vs throttle conflict** | 🟡 **Fixed, awaiting re-flash** | Was a real bug (brake ignored when throttle present). One-line fix applied; needs DBW re-flash + re-run to confirm. |
| Firmware stability under load | 🟢 Healthy | Prior intermittent freeze (USB blocking) resolved; runs all 31 tests without hanging. |
| Steering range symmetry | 🟡 Watch | Firmware design limit is asymmetric (-240 left / +250 right) but sim allows -250 left. Mismatch, not dangerous. |

**Bottom line:** the trike's control path is in good shape. Testing surfaced
**one genuine safety bug** (brake did not override throttle), which has been
fixed in firmware and now needs to be flashed and re-verified. A batch of new
corner-case tests (below) is ready to run to widen coverage.

---

## 2. What has been done & verified (real results)

### 2.1 Flaky-test root cause — DBW freeze (FIXED, verified)
- **Symptom:** tests passed/failed at random; `dbw_angle_tenths` sometimes stuck, wheel railed.
- **Root cause:** DBW logs to Native USB (`SerialUSB`). When plugged into the PC with nothing draining the buffer, `SerialUSB.print()/flush()` **blocked forever**, freezing the 100 ms control loop.
- **Fix:** made all DBW USB writes non-blocking (gate on `availableForWrite()`, removed `flush()`).
- **Result:** no freezing across all 31 tests; steering tracks everywhere. **29→30/31 stable.**

### 2.2 Per-CSV baseline reset (added, verified)
- **Problem:** each CSV inherited the previous test's wheel angle (carryover), causing false fails.
- **Fix:** `test_runner.py` now drives to a known baseline (centered + stopped + DBW-alive) at the **start of each CSV**, before its clock starts. Legitimate test setup, not a mid-test cheat.
- **Result:** eliminated carryover failures; every test starts clean.

### 2.3 Serpentine timing correction (fixed, verified)
- `28_serpentine` failed at t=5800 because it checked a full 400-tenth reversal only 1800 ms after the command — physically impossible (~2000 ms needed).
- Moved the two full-reversal checks to the physically-required time (t=6100 / t=9100), same ±band. **Now passes.**

### 2.4 Full suite result
- **30 / 31 passing** after the above. (The 31st, serpentine, then fixed as in 2.3.)

### 2.5 SAFETY BUG FOUND — brake did not override throttle (FIXED, awaiting re-flash)
- **Found by:** `32_edge_cases_break.csv`, Phase 3 (full throttle **+** full brake at once).
- **Observed:** vehicle ignored the brake and kept cruising at 130 cm/s. Assertion `sim_speed_cmPs < 20` FAILED.
- **Root cause (firmware):**
  - `Vehicle::receiveCan()` parses `desired_brake` from the Nav frame correctly...
  - ...but `Vehicle::update()` calls `throttle->update(desired_speed_cmPs, mode)` and **never passes the brake**.
  - `SpeedController::update()` only brakes when `desired_speed <= 0`. So `speed>0 + brake` → accelerate; brake byte is dead.
  - This stayed hidden historically because Nav always pairs "brake on" with "speed 0" — the two never contradicted until this adversarial test.
- **Fix applied** (`Vehicle.cpp`, before the throttle call):
  ```cpp
  // Brake-over-throttle safety: an explicit brake command must dominate any
  // throttle request. Force the stop path.
  if (desired_brake > 0) desired_speed_cmPs = 0;
  ```
- **Status:** compiles clean. **Needs DBW re-flash, then re-run `32` and `37` to confirm.**

---

## 3. Test inventory

### 3.1 Baseline regression suite (existing, 01–30 + response test)
Steering (center/lock/half/sweep/step/clamp), throttle (idle/cruise/ramp/
sustained/coast), brake (from cruise / hold / release / pulses / immediate),
combined drive+turn, serpentine, full mission. **Result: all passing** after §2.

### 3.2 Steep-turn min-time test (existing)
- `31_steep_turn_reversal.csv` — +200 right, then full reversal to -200; verifies the 400-tenth swing completes in the minimum physical time. **Passing.**

### 3.3 Edge-case break test (existing)
- `32_edge_cases_break.csv` — over-limit clamp both ways + throttle/brake conflict. Phases 1,2,4 pass; Phase 3 exposed the brake bug (§2.5), fix pending re-flash.

### 3.4 NEW corner-case tests (added this session — PENDING first run)
> These are created and ready; expected outcomes are **predictions from the
> firmware/sim code and physics**, to be confirmed on hardware after re-flash.

| File | Probes | Expected (safe) outcome |
|------|--------|-------------------------|
| `33_negative_speed_safe_stop.csv` | Invalid **negative** speed command | Treated as STOP (`speed<20`), not reverse/garbage |
| `34_steer_whipsaw_tracking.csv` | Rapid L/R reversals faster than slew | Ends at the **last** command (-250); no stale-command win / no lockup |
| `35_turn_under_braking.csv` | Steer to full lock **while braking** | Vehicle stops **and** wheel still reaches lock (steer/speed decoupled) |
| `36_overflow_speed_clamp.csv` | Absurd speed (`30000`) | Clamps to sane max, moves forward, no wrap-to-negative |
| `37_throttle_brake_conflict.csv` | Throttle **+** brake together (regression guard for §2.5) | Brake wins, `speed<20` |
| `38_sub_deadband_hold.csv` | Tiny angle below the ±5-tenth deadband | Holds near center, no chatter / no drive to lock |

---

## 4. Known issues & recommendations

1. **Brake voltage levels not distinguished (design limitation).**
   The DBW treats any `brake > 0` as ON (internal 24V-grab → 12V-hold). It does
   **not** act differently for `brake 1` (12V) vs `brake 2` (24V). If Nav needs
   selectable brake force, `desired_brake` must be wired into `SpeedController`.
   Otherwise the command spec's three values collapse to on/off — recommend
   documenting brake as on/off.

2. **Asymmetric steering limit vs simulator.**
   Firmware: `MIN_LEFT_DEGx10 = -240`, `MAX_RIGHT_DEGx10 = +250` (Settings.h).
   The simulator lets the left side reach -250. Not dangerous, but the firmware
   design limit and the sim disagree by 10 tenths on the left. Recommend
   reconciling (either -250 firmware limit, or clamp sim to -240) so tests and
   hardware agree.

3. **No explicit clamp of incoming CAN steering angle** to `[-240, +250]` was
   found in the DBW path; clamping currently happens downstream (sim/actuator).
   Consider an explicit `constrain()` on `desired_angle_DegX10` for defense in
   depth.

4. **Re-flash pending.** The brake-over-throttle fix is in source only. Flash
   the DBW before trusting Phase-3 / test 37 results.

---

## 5. How to run

```powershell
cd C:\projects\elcano-Bridge
# find the Router's COM port first (it changes on replug):
[System.IO.Ports.SerialPort]::GetPortNames()

# whole suite (auto-picks up the new 33-38 files):
python tools/run_all_tests.py COM## 

# a single test, verbose:
python tools/test_runner.py --verbose tests/37_throttle_brake_conflict.csv COM##
```

Each CSV auto-runs the per-CSV baseline reset first (centers + stops the rig),
so tests are independent regardless of order.

---

## 6. Next actions

- [ ] Re-flash DBW with the brake-over-throttle fix.
- [ ] Re-run `32` and `37` → confirm brake now beats throttle (`speed < 20`).
- [ ] Run new corner-case tests `33`–`38`; record actual results in §3.4.
- [ ] Decide on brake-level (12V/24V) handling and the -240/-250 reconciliation.
