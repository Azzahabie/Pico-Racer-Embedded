# scan_task bench tests (Buddy 5)

Hardware tests of obstacle detection, scanning and the avoidance decision
(FR-13 to FR-17, NFR-06, NFR-20). The car stays braked: the bypass drive
itself is Buddy 2's and is not built yet, so these stop at the decision.

Tester: ______  Date: ______  Firmware commit: ______
(`git log -1 --oneline`)

## Setup

- Build `make SMP=1 CONSOLE=usb_cdc` with `OBSTACLE_TEST_MODE 4` (normal), or
  `3` (scan every 3 s, no mission or buttons) for quicker repeats.
- Car on a flat floor with clear space around it; open the serial monitor.
- Mode 4: wait for `CALIBRATING -> READY`, press GP20. Between layouts press
  GP21 (pause) then GP20 (resume).
- Measure with a ruler from the **sensor face**. x = sideways (left
  negative), y = straight ahead. Note each object's width.
- For every run, keep the `[scan]` lines from `---- sweep` to `ACTION`:
  paste them into `logs/` as `layoutN_runM.txt`.

Thresholds in this build (`scan_logic.h`): D_stop 250, D_obs 400,
C_pass 120, D_rev 100 mm.

## 1. Layouts (FR-14 to FR-17)

Pass = the `ACTION` line matches. Run each layout 3 times.

| # | Layout | Expected | Run 1 | Run 2 | Run 3 | Notes |
|---|---|---|---|---|---|---|
| 1 | 100 mm box, 200 mm ahead, centred | BYPASS L or R | | | | |
| 2 | Box ahead + wall 150 mm to its right | BYPASS LEFT | | | | |
| 3 | Box ahead + wall 150 mm to its left | BYPASS RIGHT | | | | |
| 4 | 600 mm board, 200 mm ahead | BLOCKED | | | | |
| 5 | Box 80 mm ahead | REVERSE | | | | |
| 6 | Box 200 mm ahead, 200-300 mm to the right | no stop / CONTINUE | | | | |
| 7 | Box mostly left, right edge 40 mm into the path | BYPASS RIGHT | | | | |
| 8 | Box ahead, walls 150 mm right and 190 mm left | BYPASS LEFT, offset L >= 120 mm + half the box | | | | |

Also check on each bypass that the chosen side's `offset` is at least
(distance from centre to that edge of the box) + C_pass (218 mm since D12;
it was 120). A shorter offset would hit the box even if the side is right.

## 2. Width accuracy (NFR-20)

Box centred at the stated distance. 5 scans each. Pass = |error| <= 30 %
of the true width in at least 4 of 5 scans.

| True width (mm) | Distance (mm) | Scan 1 | Scan 2 | Scan 3 | Scan 4 | Scan 5 | Within 30 % | Pass (>= 4/5) |
|---|---|---|---|---|---|---|---|---|
| 50 | 200 | | | | | | /5 | |
| 90 (demo box) | 200 | 162 | 150 | 161 | 150 | 150 | 0/5 | **FAIL** (+67 to +80 %) |
| 100 | 200 | | | | | | /5 | |
| 150 | 200 | | | | | | /5 | |
| 200 | 200 | 199 | 167 | 168 | 168 | 146 | 5/5 | **PASS** (-0.5 to -27 %) |
| 90 (demo box) | 250 | 287 | 257 | 256 | 256 | 289 | 0/5 | **FAIL** (+184 to +221 %) |
| 100 | 300 | | | | | | /5 | |

Filled rows: 3 Nov 2026, mode 3, dwell scan (median of 3 per angle, 10 deg
fine steps), k = 0.080. Logs: `logs/dwell_*_k080.txt`. Details and the
reason (no single k fits both boxes) under "Results 3-4 Nov" below.

Estimates all low or all high by a similar amount point at the beam
correction `SL_BEAM_CORR_PER_1000` (80 now) in `scan_logic.h`; note it
here rather than changing it mid-test.

## 3. Detection range (NFR-06)

Mode 0 (`OBSTACLE_TEST_MODE 0`) prints mean/min/max over 20 pings. Pass =
at least 19 of 20 valid readings at each distance.

| Distance (mm) | Mean | Min | Max | OK / 20 | Pass |
|---|---|---|---|---|---|
| 50 | | | | | |
| 100 | | | | | |
| 200 | | | | | |
| 300 | | | | | |
| 400 | | | | | |

## 4. Scan time (NFR-07 budget)

From the `scan took ... ms` part of each `ACTION` line. Budget 7000 ms
(`SCAN_BUDGET_MS`, raised from 5500 for the dwell scan, 3 Nov).

Longest seen: **6541 ms** (mission, rescan after a reverse, 4 Nov); mode 3
bench 5606-6140 ms; mission first scans 4164-5726 ms.   Any `OVER`: **no**
(3-4 Nov, dwell build).

## Turning a real scan into a PC test

Each recorded sweep can become a regression test in
`tests/scan_logic_test.c`, so the decision is checked on every change.
For example, from a log

```
[scan] angle -25: 107 mm  <- obstacle
[scan] angle 0: far
[scan] ACTION: BYPASS LEFT
```

write

```c
grid_clear(grid);
set(grid, -25, 107);
set(grid,   0, SL_FAR);          /* "far" */
/* ...one set() per printed angle; angles not printed stay unmeasured */
act = run(grid, SL_D_OBS_MM, false, &prof);   /* false = no reverse tried yet */
check("bench layout 2 run 1: bypass left", SL_BYPASS_LEFT == act);
```

Run the PC tests (no board needed):

```sh
docker compose run --rm build bash -lc "cd tests && gcc -Wall -I../app_program/obstacle_task -o /tmp/sl scan_logic_test.c ../app_program/obstacle_task/scan_logic.c && /tmp/sl"
```

---

## Results so far (29 Oct 2026, bench, car braked)

Sensor: HC-SR04 on the SG90, top of the module ~10 cm above the floor
(transducer centre ~9 cm). Raw logs were in the serial monitor; paste them
into `logs/` when re-run.

### Urgent stop (NFR-18)
STOP_SCAN to brake write: 25-38 us in every run (limit 20 ms). PASS.

### Detection range (NFR-06), mode 0, 20 pings per round
| Obstacle | 40-20 cm | <= 10 cm | Result |
|---|---|---|---|
| Tall box (> 10 cm) | 20/20 at every step from ~50 cm to ~7 cm | 20/20 | PASS |
| Low box (~5 cm) | mostly 2-6/20, erratic | 20/20 | FAIL above ~10 cm |
| Nothing in front | no close readings; far wall ~1.87 m; no-echo 0 | | sensor healthy |

Low obstacles return too weak an echo to be seen at the 25 cm stop
distance. Open: the minimum obstacle height for the course (check the
brief; a 7-8 cm box at 40/30/25/20 cm pins the limit), recorded as an A5
assumption, or lower / tilt the sensor.

### Layout 1, box 200 mm wide, ~20 cm ahead (FR-14 to FR-17)
| Run | Firmware | Action | Width (true 200) | Scan time | Notes |
|---|---|---|---|---|---|
| 1 | servo 2 deg/30 ms | BYPASS LEFT | 151 | 5941 OVER | servo too slow -> 2 deg/20 ms |
| 2 | 02:00 | BYPASS RIGHT | 122 | 4804 | |
| 3 | 02:07 | BYPASS LEFT | 15 | 4818 | edge lost after the edge median: fixed (inward re-check) |
| 3 redo | 02:12 | BYPASS LEFT | 132 | 4607 | |

Action correct in every valid run. Widths read low with the theoretical
beam correction (0.132), see below.

### Width (NFR-20), mode 3, 10 cm box centred at ~19 cm
Scans 6-10: 81, 81, 82, 92, 120 mm with k = 0.132 (5/5 within 30 %).

### Beam correction chosen from both boxes (`SL_BEAM_CORR_PER_1000`)
Recomputed from the logged readings (x = d sin(theta), edges pulled in by d*k):

| k | 100 mm box (5 scans) | 200 mm box (3 runs) |
|---|---|---|
| 0.132 (theory) | 5/5 | 1/3 (-25, -39, -34 %) |
| **0.080 (chosen)** | **4/5** (+1, +1, +3, +11, +41 %) | **3/3** (-14, -28, -24 %) |
| 0 | 0/5 | 3/3 |

0.080 passes both and reads the wide box less narrow (a narrow width
shortens the bypass offset). To confirm: 5 scans each of 50/100/150/200 mm
tall boxes at 20 cm with the 03:09 build.

### Bugs found by these tests (fixed)
- Fine scan started at the outermost coarse hits: with objects either side
  it never measured the edges of the one in the path (width 0, offset too
  short). Now starts at the in-path object (`sl_fine_starts`, Test 15).
- Edge median of 3 could turn the edge hit into a miss and collapse the
  edge inward (200 mm box read 15 mm). Now re-checks inward.
- Missing avoid_cmd_mbf retried every tick and flooded the console; now
  Blocked at once (FR-18), full buffer -> Blocked after t_block.
- Mode 3 aborted every scan after one ping (no mission). Fixed.

### Still to do
Layouts 2-8, sensor-lost (unplug ECHO), Blocked -> cleared, ramp with a
clear path (no STOP_SCAN, N_hump = 4), pause mid-scan, nav_ctrl state
checks, timing with Wi-Fi streaming, and everything that needs Buddy 2's
avoid_cmd_mbf (bypass drive, full buffer, back-to-back avoidances, NFR-07).
*(3-4 Nov: the bypass drive now runs on the car, see below.)*

---

## Results 3-4 Nov 2026 (dwell scan build, k = 0.080)

Car on the bench (mode 3) and on the floor (mode 4, USB console, read
through `tools/telemetry_viewer.py`). Logs in `logs/`.

### Width (NFR-20): table in section 2
- 200 mm box passes (5/5); the 90 mm demo box fails at 200 mm (0/5) and at
  250 mm (0/5). It reads far too wide because the HC-SR04's ~+-15 deg cone
  still hears the box corner at +-25 deg (200 mm) and +-30-35 deg (250 mm).
- **No single k passes both boxes** (recomputed from the 10 logged scans):
  k 0.08 -> 0/5 and 5/5; 0.16 -> 3/5 and 1/5; 0.18-0.20 -> 5/5 and 1/5.
  Both boxes' edges land within about one 5-degree step of each other.
- **Decision: keep k = 0.080 ("option 1").** Over-reading is safe: the
  bypass swings wider (demo box: offset 261-306 mm vs 263 needed). A larger
  k would under-read wide boxes and shorten the offset (contact, as in S7).
  Cost: the reported width (`E obstacle`, `S.obs`) is 15-29 cm for a 9 cm box.
- Margin to `SL_MAX_OFFSET_MM` (400) shrinks with distance: ~95 mm at
  200 mm, 34-67 mm at 250 mm; a cluttered run at 334 mm went over (421).

### Stop distance (NFR-08), mission runs, demo box on the line
`angle 0` / closest at the first scan: **230, 231, 233, 243, 246 mm**, all
below D_stop 250 (so the scan happens at or inside the tested 200-250 mm
range). Rest position not yet tape-measured.

### Bypass drive (FR-17, FR-18, FR-19, FR-20)
- Full bypass and rejoin on the car: `logs/fr20_bypass_rejoin_2026-11-04.txt`
  (STOP_SCAN 243 mm -> BYPASS RIGHT 265 -> probe saw the box -> clear ->
  turn in -> LINE_FOUND -> Monitoring only after LINE_FOUND: FR-20).
- A second object ~50 cm left of the line made the car loop (bypass left,
  braked mid-sidestep, re-plan right, lane blocked, repeat). Two fixes in
  `obstacle_task.c` / `scan_logic.c` (PC Test 19), **not yet re-run on the car**:
  - the sidestep lane look must hold leg + D_stop + 50 mm (`sl_lane_clear`),
    else LOOK 1 -> motion pivots back -> other side;
  - after `SL_MAX_BYPASS_PLANS` (3) bypass plans in one avoidance: Blocked.
- Side look read 79 mm on the 265 mm bypass: the car passed ~30 mm closer
  than planned. Cause found below.

### Right encoder gives 0.63x counts per turn (all motion, Buddy 2)
Hand push, hand turns and the bench open-loop test all show the right
encoder at ~1590 counts per wheel turn vs 2535 on the left (not wiring:
steady ratio, `missed` ~0.6 %). Distances read ~20 % short and the right
wheel ran ~1.6x fast off the line. Scaled to left-wheel units in
`motor_task.c` (`MOTION_COUNTS_PER_REV_R`); see
`docs/motion/right_encoder_scaling.md`. Re-check pivots and the bypass
margin after Buddy 2 re-checks motion.

### Counted bypass trials (FR-18, FR-19, NFR-07, NFR-08)
| Batch | Image | Bypasses rejoined | Contact | STOP_SCAN -> line found | Stop distance | Log |
|---|---|---|---|---|---|---|
| 1 (10 trials) | 03:32 (IMU heading, 1.5 s line average) | 6 of 8 (2 runs never reached a bypass) | 4/10, on the turn back in (7, 8, 9, 10) | 27.9-29.1 s | 223-248 mm | `logs/trials_batch1_2026-11-04.txt` |
| 2 (partial) | 04:32 (line direction over 400 mm, 80 mm margin) | 3 of 3 (box seen at the first look every time, 151-261 mm) | 0/3 | 28.2-33.4 s | 245-252 mm | `logs/trials_batch2_2026-11-04.txt` |

Batch 1 trials 9-10 never saw the box and lost the line: the line direction
was averaged over 1.5 s (~7 cm at line-following speed) while line following
wandered up to 28 deg, so the bypass ran ~20 deg off. Fixed at 04:32. To
finish: one full 10-trial batch on the 04:32 image, with contact recorded.

### IMU
Intermittent: calibration sometimes reads x = y = z = 5987 mg with false
BUMP / tilt faults; passes after re-seating and a reboot. Wiring (I2C) to check.

### Logs
| File | What |
|---|---|
| `dwell_200mm_90mmbox_k080.txt` | NFR-20, demo box at 200 mm, 5 scans |
| `dwell_200mm_200mmbox_k080.txt` | NFR-20, 200 mm box at 200 mm, 5 scans |
| `dwell_250mm_90mmbox_k080.txt` | NFR-20, demo box at 250 mm, 5 scans |
| `dwell_334mm_90mmbox_k080_cluttered.txt` | invalid setup (clutter), kept: offset over 400 |
| `fr20_bypass_rejoin_2026-11-04.txt` | FR-18/19/20, full bypass and line rejoin |
| `imu_bypass_left_right_2026-11-04.txt` | First IMU-heading bypasses that rejoined (left and right) |
| `trials_batch1_2026-11-04.txt` | Counted trials batch 1 (10 trials, 03:32 image) with the tester's contact answers |
| `trials_batch2_2026-11-04.txt` | Bypass runs on the 04:32 image (3 real bypasses) |
