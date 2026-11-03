# Right encoder scaling — for Buddy 2

*Written 4 Nov 2026 by Buddy 5 (obstacle task), during bypass testing on
branch `obstacle_task`. Not yet run on the car with the change in. Please
review before it goes into `master`.*

## 1. Summary

The **right wheel's encoder gives about 1590 counts per wheel turn; the left
gives 2535** (`MOTION_COUNTS_PER_REV`). Both wheels physically turn at about
the same speed for the same duty. The right motor most likely has a
different gearbox (1590 / 2535 = 0.63, close to 30 / 48 = 0.625).
**Please check the two motors' labels.**

motion_task used one `MOTION_COUNTS_PER_REV` for both wheels, so:

- **Distance** (the average of both wheels) read about 20 % short: a hand
  push of 1000 mm read 680-783 mm.
- **Speed PID** made the *counts* equal, so the right wheel actually spun
  about 1.6x faster than the left. Off the line (bypass legs, reverse) the
  car steers off; on the line, line following hid it.
- **Pivots and bypass sidesteps** came out wrong. The IMU logged 70°
  pivots as 67-69° out but 59-60° back, and a side look found the box
  30 mm closer than planned.

**The change:** the right encoder's count is scaled to left-wheel units the
moment it is read (`tick_encoders`). Everything after that (PID, distance_state,
pivots, bypass, reverse) is unchanged and sees one unit for both wheels.

## 2. Evidence (car, 4 Nov 2026, USB console)

| Test | Left | Right | Right / Left |
|---|---|---|---|
| Hand push along a tape, 5 pushes (`count` lines) | 839-968 mm-equivalent | 509-599 | **0.60-0.62** |
| Right wheel turned by hand, wheels off the ground | — | single turns: **1582, 1583, 1595, 1598** (first 1646, with slop) | — |
| Earlier 5-turn hand test | — | 8180 counts in 5 turns (= 1636 per turn) | — |
| Bench open-loop (`MOTION_BENCH 1`), 35 % duty, 3 runs: `L fwd` / `R fwd` | 5018-5049 | 3101-3111 | **0.62** |
| Same, `L rev` / `R rev` | 5020-5042 | 3571-3587 | 0.71 |
| Same, `both fwd` | 4986-4991 | 3063-3069 | **0.62** |
| Dead band | 50-100 per-mille | 75-100 | — |

It is **not a wiring fault**: the ratio is the same at hand speed and at
35 % duty, run after run (R fwd 3101 / 3107 / 3111), no `encoder fault`, and
`missed` only grows about 0.6 % of the counts. Physical speed at 35 %:
left 3345 / 2535 = 1.32 turns/s, right 2071 / 1590 = 1.30 turns/s.

The 1590 is the mean of the clean single turns. **Refine it** with a careful
10-turn count (section 6) before relying on it.

## 3. What changed

### `app_program/motor_task/motion_cfg.h`

```c
#define MOTION_COUNTS_PER_REV_R (1590)   /* right wheel, counts per turn (x4) */
```

`MOTION_COUNTS_PER_REV` (2535) stays the one unit everything is in.
**Set `MOTION_COUNTS_PER_REV_R` equal to `MOTION_COUNTS_PER_REV` to switch
the scaling off.** The conversion then compiles to `return raw;`.

### `app_program/motor_task/motor_task.c`

- `motion_ctx_t` gets `raw_pos_r`: the right total as the encoder gives it.
  `pos_r` is now in left-wheel units.
- New `right_to_left_units(raw)`: `raw x 2535 / 1590`, 64-bit, rounded to
  nearest (half away from zero, so forward and back are symmetric).
  Truncating instead made the right wheel lag by up to 1.6 units, which
  read as a turn.
- `tick_encoders()`: the right total is scaled first, then `d_r` is taken
  from the scaled total. So rounding never builds up over a run, the same
  way `pos_l` works.

```c
p_ctx->raw_pos_r += motion_count_delta(now_r, p_ctx->prev_r);
p_ctx->prev_r     = now_r;
{
    int32_t scaled = right_to_left_units(p_ctx->raw_pos_r);
    p_ctx->d_r   = scaled - p_ctx->pos_r;
    p_ctx->pos_r = scaled;
}
```

Untouched: `encoder.c`, the per-phase edge counts and stall check
(`e_ra`/`e_rb` are raw edges), `motion_lib.c`, the statechart, all gains.

### `tests/motion_task_sim/sim_motion.c`

- The simulated right encoder now gives `MOTION_COUNTS_PER_REV_R` counts
  per turn, like the car. The encoder travels with its motor in the
  plugs-swapped scenario.
- Two checks compared the wheels *exactly*. They now allow **one right count
  of resolution**, computed from the constants (0 when the scaling is off):
  - straight line: turn rate within ±`sim_turn_tol()` (1 deg/s; it read −1);
  - reverse twice, PID reset: right first-tick duty within
    `SIM_R_DUTY_TOL` = 2 units x (Kp 8 + Ki 1) = 18 per-mille (it read
    −72 vs −90; the left matched exactly). A carried-over integrator
    would be far larger.

## 4. Test results

| Suite | Result |
|---|---|
| `make -C tests` (all host tests, Docker) | **all pass**: motion_task sim 269,817 checks + Week 6 build 380,987, 29/29 scenarios, motion_lib, motion_sm, encoder, motor, wdt, imu, line, barcode |
| Same sim with `MOTION_COUNTS_PER_REV_R` = 2535 (scaling off) | all pass, **269,950 checks, identical to `master`** (tolerances back to exact) |
| `tests/scan_logic_test.c` | all pass |
| Firmware `make SMP=1 CONSOLE=usb_cdc` | builds, no warnings |
| **On the car** | **not yet run** |

## 5. What to watch for on the car

1. **Right speed loop gain is ~1.6x higher.** Each right count now
   becomes 1.6 units of measured speed, so the same Kp/Ki act harder on
   the right wheel. Watch for the right wheel hunting or buzzing at low
   speed. If it does, a separate right Kp (Kp x 1590 / 2535) is the fix.
2. **Right speed reading is coarser.** It's still the same hardware
   resolution, just shown in finer units. Expect ±1 deg/s noise in
   `distance_state` turn and small first-tick duty steps. That's normal.
3. **Line following was tuned with the mismatch in.** The right wheel used
   to spin 1.6x faster at the same target. Re-check line following, line
   search and junction turns (`MOTION_LS_SWEEP_DEG`, base speed, steering
   gain).
4. **Pivot and bypass tuning was tuned with the mismatch in.** Re-check
   `MOTION_BYPASS_PIVOT_PCT` (90 %, set in S7 attempt 3 "90 deg turned
   ~100 by eye") with the IMU `turned ... deg` lines. It may now want 100 %.
5. **Bench test printouts** (`STEP ... moved L.. R..`) now show the right in
   left-wheel units. At equal duty L and R should now read about the same
   (it was R = 0.62 x L).
6. **The UART-console build** (left encoder off, GP0/GP1 are the console)
   is unaffected by this change. It's still "distance is half the right
   wheel", now in scaled units.

## 6. How to re-measure `MOTION_COUNTS_PER_REV_R`

Normal USB image, mission not running, wheels off the ground:

1. Put a tape marker on the right tyre.
2. Turn the wheel by hand exactly **10 turns**, slowly, counting out loud.
3. Read the change in the raw right count. Note that after this change,
   `count L.. R..` prints the **scaled** value, so either
   - divide the printed change by 10 and multiply by 1590 / 2535, or
   - temporarily set `MOTION_COUNTS_PER_REV_R` to 2535 to see raw counts.
4. counts / 10 = `MOTION_COUNTS_PER_REV_R`. Repeat 2-3 times.

Do the same on the left to confirm 2535 (`docs/motion/results.md`: 2 trials).

## 7. Next steps (suggested order)

1. Check the motor labels; refine 1590 (section 6).
2. **NFR-04 straight trials**: push or drive 1000 mm, 5x. Both wheels should
   now agree within a few percent; then calibrate `MOTION_WHEEL_DIA_MM`.
3. Pivot accuracy: `MOTION_BYPASS_PIVOT_PCT` and line-search sweep (point 4 above).
4. Re-check line following end to end.
5. Only then: Buddy 5 re-tunes the bypass margin (`SL_MARGIN_MM`, obstacle
   side) from measured sidesteps.

If a matching motor is fitted instead, set `MOTION_COUNTS_PER_REV_R` back to
2535 and everything returns to the old behaviour.

## 8. Related obstacle-side changes on the same branch (FYI)

Buddy 5's code only (`app_program/obstacle_task/`), nothing in motion:

- **Sidestep lane check.** scan_task now requires the lane look to hold
  leg + D_stop (250) + 50 mm. Otherwise it replies `LOOK 1` (unknown),
  which your `byp_look_done()` already treats as blocked, so motion pivots
  back and raises `SIDE_BLOCKED` as designed. Reason: your lane test
  (leg + `MOTION_BYPASS_LANE_MARGIN_MM` 205, capped at 511) passed a 559 mm
  lane that Avoiding's 250 mm stop then interrupted, and the car looped.
  A compile-time check ties scan_task's leg factor to
  `MOTION_BYPASS_SIDE_LEG_X1000`.
- **Retry limit.** After 3 bypass plans in one avoidance, scan_task goes to
  Blocked and tells nav, instead of re-planning forever.

Worth discussing: `MOTION_BYPASS_LANE_MARGIN_MM` (205) is smaller than D_stop
(250), so motion's own lane test can accept a lane the car will brake in.

More obstacle-side changes, 4 Nov (after the first floor bypass runs):

- **Bypass clearance 218 -> 248 mm**: new `SL_PASS_MARGIN_MM` 80 (C_pass only;
  the in-path corridor keeps `SL_MARGIN_MM` 50). With 218 the first probe
  found the box at 127 mm, the forward pings read 95 mm (< D_rev), and
  scan_task re-planned while the car was turned 45° toward the box. The car
  touched the box, then planned the next bypass from that turned pose and
  circled the box. With 248, the first probe read 239 mm and nothing touched.
- **Side switch timing**: `switch_side()` now waits 30 ms after STOP_SCAN
  before sending the new BYPASS. Your statechart takes a BYPASS only in
  Braked, and in `BYP_PH_SWITCH` the state is still Bypass, so the
  command sent in the same microsecond as the stop pulse was dropped and
  the switch phase timed out after 5 s (first time this path ran on a car).

## 9. Second motion change: the rejoin sees a confirmed black (4 Nov)

`motor_task.c`, `BYP_PH_STEP_IN`: the drive back toward the line now also
stops on `line_state_dark_ms(word) < MOTION_LF_SEEN_MS` (30 ms), not only on
`rl != LINE_RL_CENTRED`.

**Why (car, 4 Nov, 02:56):** after a clean bypass the car turned in at about
60° and crossed the line. line_task confirmed black (`[line] line found`;
0.5 s later `dark_ms=223`), but the filtered `rl` never left 00: no
`[line] RL=..` line in that window. STEP_IN only looked at `rl`, so the car
drove on to leg + `MOTION_BYPASS_IN_EXTRA_MM` (1160 -> 1615 mm), pivoted with
the line behind it, and ended in LINE_LOST.

It's the same test `line_inputs()` already uses for `b_line_seen`, so the
rejoin is no longer stricter than line following. All host tests pass. The
simulator has no scenario where only `dark_ms` shows the line during STEP_IN,
so this case is covered by the car run only. **Worth adding to
`tests/motion_task_sim`.**

Worth a look on your side: why `rl` stays 00 on an angled crossing while
the dark timer sees black. It may also affect line search and junctions.

## 10. Third motion change: the bypass steers by the IMU heading (4 Nov)

**Problem (car, 4 Nov, 03:11 and 03:13).** Line following zig-zags ±20-30°,
so the car stops for the box at an angle to the line: the IMU read **22°**
and **29° left** at the two scans. Every bypass phase measured its turns and
legs on the encoders *from that pose*, so the whole rectangle was rotated:
- right bypass: only reached the line at the very end of the maximum leg
  (`dark_ms=0` as it gave up);
- left bypass: back at the line after ~30 mm, then the final 70° "onto the
  heading" pivot left it 30° off the line and lost it.

The return distance was never the problem; the reference direction was.

**Change (`motor_task.c`, `motion_cfg.h` `MOTION_HD_*`).** `hd_tick()` runs
every tick before `man_drive`:
- **Line direction** = running average of `ev_heading_deg()` (imu_task, left
  +) over Line Following, 1.5 s (`MOTION_HD_TAU_TICKS`), trusted after 1 s
  (`MOTION_HD_MIN_TICKS`). Restarted on a jump > 30°/tick (mission-start
  reset); forgotten in Idle, Turning, Motion Fault and whenever the IMU
  word's FAULT bit is set.
- **Sideways distance** from the line, summed every tick off the line:
  (d_l + d_r) x sin(heading - line).
- **When the heading is valid (`b_hd_ok`):**
  - SIDESTEP ends at the planned sideways distance (current + offset, so a
    re-plan off the line still works), capped at 2 x leg;
  - every pivot turns to a heading relative to the line: OUT +70,
    LANE_BACK / PARALLEL / PROBE_OUT / ALONG 0, PROBE_IN -45, TURN_IN -70
    (x side), stopping `MOTION_HD_LEAD_DEG` (4°) early for the overshoot,
    with 2x the old encoder count as a backstop;
  - STEP_IN also ends when the line sensors (125 mm ahead of the axle) are
    back over the line's position, capped at 2 x (leg + extra);
  - all line-sensor checks are unchanged and still win.
- **Otherwise every phase runs exactly as before** (encoders only).
  `MOTION_HD_ENABLE 0` builds the old bypass.
- New console line on every phase change, e.g.
  `[motion] bypass PARALLEL: 312 mm out, heading +66 deg to the line, IMU`
  (or `encoders only`).

**Tests.** New `scenario_bypass_heading` in `tests/motion_task_sim`: learn
the line, stop **±25°** off it, bypass LEFT and RIGHT with the line sensors
never seeing the line. Result: ends **4° off** (the lead) and the axle
**0-1 mm** from the line in all four cases. With `MOTION_HD_ENABLE=0` the
same scenario ends **21-29° off** (17 failures), which matches the car. The
IMU-fault case falls back to `encoders only`. The existing bypass
scenarios learn the line for only 3 ticks, so they still test the encoder
path unchanged. All host tests pass. **Not yet run on the car.**

**Risks to check on the car:**
1. The IMU on this car is intermittent (calibration sometimes reads
   x = y = z = 5987 mg). A bad heading without the FAULT bit set would
   steer the bypass wrong; the 30° jump guard and the 2x encoder backstop
   limit the damage. Watch for `IMU` lines with silly headings.
2. The 4° lead is a guess. If pivots end consistently short or long in
   the `bypass ...` lines, tune `MOTION_HD_LEAD_DEG`.
3. A curved line before the box would bias the averaged direction.

**Update 04:32 (after counted trials batch 1):** the line direction is now
averaged over **distance** (`MOTION_HD_LINE_MM` 400, trusted after
`MOTION_HD_MIN_MM` 300 of Line Following) instead of 1.5 s. At line-following
speed (~4.75 cm/s) 1.5 s was only ~7 cm, and line following wanders slowly
(2° R → 28° L over 8 s), so in trials 9-10 the direction came out ~20° off:
the probes missed the box, the car turned in onto it and missed the line.
STEP_IN also drives `MOTION_HD_PAST_MM` (80) past the estimated line before
giving up, since the line was seen 4-30 mm before the estimate in the good
trials. New sim checks: a slow ±20° wander is learned to 0°; a line 50 mm
beyond the estimate is found. Trials: `tests/scan_bench/logs/trials_batch1_2026-11-04.txt`.
