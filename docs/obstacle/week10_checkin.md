# Week 10 check-in: what Buddy 5 can show

scan_task (ultrasonic obstacle detection, scanning and avoidance decision,
`app_program/obstacle_task/`). Numbers come from the 29 Oct 2026 bench session
(car braked) in `tests/scan_bench/README.md`, from
`docs/obstacle/week13_representation.md`, and from the 1 Nov 2026 integration
on the car (stage S7, `docs/INTEGRATION.md` §4, §5 and decisions D12-D14).

## 1. Against the Week 10 demo table

| Test | A | B | C | What we have | Band |
|---|---|---|---|---|---|
| Obstacle detection rate | 100% | >= 95% | >= 90% | Tall box (> 10 cm): **20/20 pings** at every step from ~50 cm to ~7 cm. Low box (~5 cm): only **2-6/20** above ~10 cm. | **A** for tall obstacles; low obstacles open |
| Width estimation error | <= 10% | <= 20% | <= 30% | k = 0.080: 100 mm box **+1, +1, +3, +11, +41 %**; 200 mm box **-14, -28, -24 %**. 7/8 within 30 %, 3/8 within 10 %. | ~**C** (some B) |
| Correct direction chosen | 100% | >= 95% | >= 90% | Layout 1 (200 mm box, centred, 20 cm ahead): **4/4** valid runs correct BYPASS. On the car (S7) the side is chosen by clearance, and a blocked lane switches to the other side (D14). | 100 %, one layout only |
| Line reacquired | 100% | >= 95% | >= 90% | **Works on the car**: the probe bypass passed boxes of unknown depth and rejoined the line in S7 attempts 3-4. No counted runs yet. | Not banded: needs counted runs |
| Time to bypass the obstacle | < 3 s | < 5 s | < 7 s | Scan alone **3.9-5.3 s**; whole bypass **~20-30 s** on the car. Sim: 200 mm box **26.4 s**, 500 mm box **34.5 s**. | **Not met** (NFR-07), see §5 |

## 2. Other numbers worth showing

- **Urgent stop (NFR-18):** STOP_SCAN to brake write **25-38 µs** in every
  run, against a 20 ms limit. In the integrated car (S2): ≤ 314 µs with a
  stand-in sender, 217 µs with the real senders.
- **Thresholds** (`scan_logic.h`): D_stop 250 mm, D_obs 400 mm, D_rev 100 mm,
  5° scan step over -45..+45°.
- **Car geometry** (measured 1 Nov, D12): 125 mm wide (half width 63 mm),
  axle to front 155 mm, axle to back 70 mm.
- **Clearances** (D12): C_pass **218 mm** (pivot sweep 168 + margin 50),
  path corridor 113 mm (half width + margin), max bypass offset **400 mm**
  (was 300; a 200 mm box now fits).
- **Debounce:** 2 close readings to stop, **4 on a hump** (Buddy 4's
  `hump_state` word, integration), 3 clear readings to leave Blocked, 3
  no-echo pings to enter Sensor Lost.
- **motion_task timing with scan_task running:** tick 10 ms ± 0.2, 0 lost
  ticks, watchdog gap <= 10.4 ms (integration with Buddy 2).
- **Scan time:** 4.6-5.0 s on the bench, 3.9-5.3 s on the car.

## 3. Integration on the car (S7, 1 Nov)

`avoid_cmd_mbf` is now bound (`98f7dd2`): scan_task's REVERSE and BYPASS
commands reach motion_task instead of failing closed. Three changes came out
of attempts 1-3 (`ce80cad`):

| Change | Why | Result |
|---|---|---|
| **C_pass = pivot sweep + margin** (218 mm, was 120) (D12) | The bypass pivots beside the obstacle; with 120 mm the car's corners hit it (attempt 1, runs 3, 4, 8). | Clears the box on the pivot |
| **Pivot-aware re-plan** (D13): stop below D_stop while driving straight, below D_rev while still or pivoting (distance changed ≤ 2 mm since the last ping) | Attempt 1 re-planned the bypass **15 times** on its own pivots. | Attempt 2: **0** false re-plans |
| **Bypass looks** (D14): `SCAN_FLG_LOOK` (servo 45° toward the box while the car has pivoted 45°, so the look is square-on), `SCAN_FLG_LOOK_AHEAD` (servo 0°, along the sidestep lane). Both answer with the new avoid op **`MOTION_AVOID_OP_LOOK` (3)**: median reading, 1 = no echo, 511 = nothing in range. | At 45° incidence the box reflected the sound away (echo test, 1 Nov). The probe bypass needs these looks to handle obstacles of unknown size. | Probe bypass works (attempts 3-4) |
| **Side switch** (D14): `SCAN_FLG_SIDE_BLOCKED`. The lane was blocked and the car is back on its heading, so scan re-plans the other side **once** from the original profile (`sl_other_side`), else Blocked | Attempt 3 re-planned in the car's turned frame when a blocker sat outside the scan's view. | Built; host test 18 |

The bypass drive itself is Buddy 2's probe bypass (`44b2955`, `bf892b5`):
sidestep at 70°, then 250 mm and 100 mm steps alongside the box with a
square-on look after each. "Passed" means a reading ≥ 450 mm after the box
was seen once, followed by 120 mm more before turning back in to find the
line. It gives up at 1.5 m alongside (Line Search, then LINE_LOST).

## 4. Calibration: beam correction `SL_BEAM_CORR_PER_1000`

Recomputed from the logged readings (x = d sin θ, edges pulled in by d·k):

| k | 100 mm box (5 scans) within 30 % | 200 mm box (3 runs) within 30 % |
|---|---|---|
| 0.132 (theory) | 5/5 | 1/3 (-25, -39, -34 %) |
| **0.080 (chosen)** | **4/5** (+1, +1, +3, +11, +41 %) | **3/3** (-14, -28, -24 %) |
| 0 | 0/5 | 3/3 |

0.080 passes both boxes and reads the wide box less narrow (a narrow width
shortens the bypass offset).

### Bench runs, layout 1 (200 mm box, ~20 cm ahead)

| Run | Firmware | Action | Width (true 200) | Scan time | Notes |
|---|---|---|---|---|---|
| 1 | servo 2°/30 ms | BYPASS LEFT | 151 | 5941 OVER | servo too slow -> 2°/20 ms |
| 2 | 02:00 | BYPASS RIGHT | 122 | 4804 | |
| 3 | 02:07 | BYPASS LEFT | 15 | 4818 | edge lost after median: fixed |
| 3 redo | 02:12 | BYPASS LEFT | 132 | 4607 | |

### Bugs found by testing (all fixed)

1. Fine scan started at the outermost coarse hits: with objects either side
   it never measured the box in the path (width 0, bypass offset too short).
   PC Test 15.
2. Edge median of 3 could collapse the edge inward (200 mm box read 15 mm).
   Now re-checks inward.
3. Missing `avoid_cmd_mbf` flooded the console; now Blocked at once (FR-18).
   (`avoid_cmd_mbf` is now bound, so this only applies without motion.)
4. Mode 3 aborted every scan after one ping. Fixed.
5. *(car, S7)* C_pass of 120 mm let the corners hit the box on the pivot.
   Now 218 mm (D12). PC Test 17.
6. *(car, S7)* The car's own pivots set off STOP_SCAN and re-planned the
   bypass 15 times. Now uses D_rev while still or pivoting (D13). PC Test 16.
7. *(car, S7)* A blocked lane re-planned in the turned frame. Now re-plans
   the other side from the original profile (D14). PC Test 18.

### Repeatable on the spot

- **Live:** `OBSTACLE_TEST_MODE 3` scans every 3 s with no mission or
  buttons. Place a box and read the `[scan] ... ACTION` line and width.
- **Range:** `OBSTACLE_TEST_MODE 0` prints mean/min/max over 20 pings.
- **PC regression tests** (no board), **18 test groups**. Test 16 covers
  the D13 stop distance, Test 17 the C_pass and offset, and Test 18 the
  look reply. The file is not in `tests/Makefile`, so run it by hand:

  ```sh
  docker compose run --rm build bash -lc "cd tests && gcc -Wall -I../app_program/obstacle_task -o /tmp/sl scan_logic_test.c ../app_program/obstacle_task/scan_logic.c && /tmp/sl"
  ```

## 5. Gaps to close before the check-in (priority order)

1. **Time to bypass (NFR-07).** Scan 3.9-5.3 s plus a 20-30 s probe bypass,
   against < 7 s for C. This cannot be met at the current speeds. Report it
   as a measured shortfall (`docs/INTEGRATION.md` §8 item 3) and give the
   reason: robustness for an obstacle of unknown size was chosen over speed
   (D14).
2. **Count the bypass runs on the car**: "line reacquired" and the full
   bypass time over several runs (e.g. 5 per box size), with the viewer logs
   saved. Attempts 3-4 show it works but give no rate.
3. **Layouts 2-8, 3 runs each** (`tests/scan_bench/README.md` §1). The left
   and right cases (2, 3, 7, 8) are what prove "correct direction". Include
   one blocked-lane run to show the side switch (D14) on the car.
4. **Width table with the 0.080 build**: 5 scans each at 50/100/150/200 mm,
   20 cm. Current 0.080 figures are recomputed from 0.132 logs, not live.
5. **Save raw logs** to `tests/scan_bench/logs/layoutN_runM.txt`; it is empty.
6. **Low obstacles:** state a minimum obstacle height (assumption A5) or
   lower / tilt the sensor.
7. **Tests:** add `tests/scan_logic_test.c` to `tests/Makefile`, and cover
   the LOOK op in `tests/host/test_messages.c` (`docs/INTEGRATION.md` §8
   item 9).
