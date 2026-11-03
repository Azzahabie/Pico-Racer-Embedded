# Obstacle task (Buddy 5): FR / NFR status

Status of scan_task (`app_program/obstacle_task/`) against the requirements
in the **Design Review** (Week 6, group IS-24), checked 3 Nov 2026 on branch
`obstacle_task`, **updated 4 Nov 2026 (04:45)** after the car tests of 3-4 Nov (§6). Evidence comes from `docs/obstacle/week10_checkin.md`,
`docs/obstacle/week13_representation.md`, `docs/INTEGRATION.md` §4 (S7) and
`tests/scan_bench/README.md`. The PC tests were re-run on 3 Nov:
`tests/scan_logic_test.c`, 18 test groups, **all pass**. 4 Nov: 19 groups
(new Test 19, lane check), all pass, and now part of `make -C tests`.

**Summary (4 Nov, 04:45):** every requirement's behaviour is implemented.
Against the exact wording:

| Status | Requirements |
|---|---|
| ✅ Met, with evidence (7) | FR-13, FR-14, FR-15, FR-16, FR-20, NFR-17, NFR-18 |
| ⚠️ Implemented, needs more trials (4) | FR-17 (layouts 2-8), FR-18 and FR-19 (3/3 on the final image, need 10), NFR-08 (trial 8 open, tape measure) |
| ⚠️ Data in, write-up / range table missing (2) | NFR-06 (range table), NFR-16 (layouts + range tables) |
| ❌ Knowingly not met, with reasons (2) | NFR-07 on time (27.9-33.4 s vs 7 s, D14); NFR-20 for the 90 mm box (reads 150-162 mm; over-reading kept because it is the safe side) |

FR-18/FR-19: batch 1 (03:32 image) had contact on the turn-in in 4/10, cause
found and fixed at 04:32; batch 2 on the fixed image: 3/3 contact-free and
rejoined. **None of the 3-4 Nov code changes are committed yet**, and the
motion changes need Buddy 2's review (`docs/motion/right_encoder_scaling.md`).

## 0. Demo obstacle (estimated size)

| Dimension | Size | What it means for scan_task |
|---|---|---|
| Height | 19-20 cm | Well above the ~10 cm where detection is reliable (20/20 pings from ~7 to ~50 cm). The low-obstacle gap (~5 cm, 2-6/20) does not apply to it |
| Width | 9 cm | Read 150-162 mm at 20 cm on the car (NFR-20). Centred, the bypass offset is the read edge + C_pass 248 mm: **~320 mm** measured (273-363 mm in the trials), inside `SL_MAX_OFFSET_MM` 400 |
| Length (depth) | 13 cm | The scan cannot see depth. The probe bypass steps 250 mm and then 100 mm alongside, so one step should clear it. It then drives 120 mm more before turning back in. The 1.5 m give-up is far away |

Use this box for the counted trials below: NFR-06 at "≥ 9 cm wide", and NFR-20
at "90 mm box, 200 mm".

## 1. Functional requirements

| Req | Wording (shortened) | Verdict | Evidence / gap |
|---|---|---|---|
| FR-13 | Detect obstacles ahead using ultrasonic | ✅ Met | 20/20 pings for tall boxes. Boxes around 5 cm high are often missed |
| FR-14 | Coarse scan before choosing an avoidance move | ✅ Met | Monitoring → STOP_SCAN → Coarse Scan, always in that order |
| FR-15 | Finer scan of the area once the coarse scan confirms an obstacle | ✅ Met | The fine scan starts from the coarse edges (PC test 15) |
| FR-16 | Estimate position, width, closest point, and left and right clearance | ✅ Met | All five values are in `sl_profile_t`. How accurate they are is NFR-20 |
| FR-17 | Choose stop, continue, turn or reverse from the profile | ⚠️ Built, partly proven | All four actions exist. On the car only layout 1 was tested (4/4). Layouts 2-8 haven't been run |
| FR-18 | Bypass **without touching** the obstacle, otherwise stop | ⚠️ **Counted, not yet passing** | S7 attempt 1 hit the box (C_pass 218 fix, D12). **Batch 1 (4 Nov, 03:32 image, 10 trials): contact in 4/10** (7, 8, 9, 10), always on the turn back in; 9-10 never saw the box (line direction ~20° off). Fixed at 04:32 (line direction over 400 mm). **Batch 2 (04:32 image): 3/3 real bypasses contact-free and rejoined, box seen every time.** Needs a full 10-trial batch on the 04:32 image (`logs/trials_batch1_*`, `logs/trials_batch2_*`) |
| FR-19 | Search for the line after the bypass and report whether it was found | ⚠️ **Counted, improving** | Line Search raises LINE_FOUND or LINE_LOST (shared with Buddy 2). Batch 1: line found in **6 of 8** bypasses (9-10 LINE_LOST, both reported correctly). Batch 2 (04:32 image): **3 of 3** found. Needs the full 10-trial batch |
| FR-20 | Resume line following only after the line is found | ✅ Met | Avoiding returns to Monitoring only on LINE_FOUND. Recorded on the car 4 Nov: `tests/scan_bench/logs/fr20_bypass_rejoin_2026-11-04.txt` (`LINE_FOUND`, then `AVOIDING -> MONITORING`) |

## 2. Non-functional requirements

| Req | Wording (shortened) | Verdict | Evidence / gap |
|---|---|---|---|
| NFR-06 | Detect an obstacle ≥ **[X] cm** wide at **[X-Y] cm**, 19/20 trials | ⚠️ **Data in, range table missing** | Proposed wording from the 29 Oct mode 0 data: **"≥ 9 cm wide and ≥ 10 cm tall, at 7-50 cm, 19/20"** (tall boxes read 20/20 at every step from ~50 to ~7 cm; ~5 cm boxes fail above ~10 cm). The demo box (9 × 20 cm) is inside that. Still to do: mode 0 with the demo box at 50, 100, 200, 300, 400 mm to fill `tests/scan_bench/README.md` §3 |
| **NFR-07** | Pick the larger-clearance side, bypass with no contact, find the line again in **9 of 10 trials, each ≤ 7 s** (STOP_SCAN → line found) | ❌ **Not met** | **Time: measured 27.9-33.4 s** STOP_SCAN → line found (batches 1-2, 4 Nov); the scan alone is 4.7-6.6 s, so 7 s cannot be met (known since Tribunal 1, D14). **Success rate:** batch 1 (03:32 image) 5/10 contact-free with line found (1, 2, 3, 5, 6); batch 2 (04:32 image) 3/3 contact-free with line found. The larger-clearance side choice is built (both sides open in every trial; the side with the smaller offset is taken) |
| NFR-08 | Come to rest before touching an obstacle at the set stopping distance and speed | ⚠️ **Mostly met** | 4 Nov, demo box on the line: closest at the first scan **223-252 mm in every bypass of batches 1-2** (plus 230-246 mm earlier), no contact at the stop. **Open:** batch 1 trial 8 reported contact with no stop or scan in the log (possible missed detection, to repeat), and one stray 367 mm detection (width 0) in batch 2. Tape-measure the rest position in a few runs |
| NFR-16 | Documented test procedure **with recorded results** | ⚠️ Mostly done | `tests/scan_bench/README.md`: width table (§2), scan time (§4), "Results 3-4 Nov" with the counted-trial table; **8 logs** in `tests/scan_bench/logs/` (width ×4, FR-20 rejoin, IMU bypasses, trial batches 1-2). Still blank: layouts 2-8 (§1), range table (§3) |
| NFR-17 | Messages match the agreed definitions, and the interface tests pass | ✅ Met | 4 Nov: 7 LOOK-op checks added to `tests/host/test_messages.c`, using the raw words motion logged on the car (0x1ff3 = 511, 0x04f3 = 79) and the new LOOK 1 reply; **257 checks pass**. It and `scan_logic_test.c` now run in `make -C tests` |
| NFR-18 | Motor stop within 20 ms of STOP_SCAN | ✅ Met | 25-38 µs on the bench, 217-314 µs in the integrated car (S2) |
| NFR-20 | Width within 30 % in 4 of 5 scans at **[X] cm** | ⚠️ **Partly met (live, 3 Nov)** | At 20 cm, k = 0.080: **200 mm box 5/5 PASS**; **90 mm demo box 0/5 FAIL** (150-162 mm), and 0/5 at 25 cm (256-289 mm). No single k passes both (sweep in `tests/scan_bench/README.md`). Kept k = 0.080 on purpose: over-reading only widens the bypass (safe), under-reading would shorten it. **Wording to report: [X] = 20 cm; met for 200 mm boxes (5/5), not met for the 90 mm box (0/5), accepted as a known limit of the HC-SR04 beam** |

## 3. What the tribunal predicted

The Tribunal 1 examiner flagged three risks in the NFR-07 claim: the 150 ms
servo settling time was a guess, there had been no end-to-end timed bypass,
and "a re-plan during Avoiding adds another full scan, so the worst case may
exceed 7 s." All three turned out to matter. For the report: the risk was
identified at the tribunal, then measured, and speed was traded for
robustness on obstacles of unknown size on purpose (D14).

## 4. To close the gaps (priority order, 4 Nov 04:45)

1. **Hardware first:** re-seat the IMU wiring (calibration read x = y = z =
   5987 again at 04:41) and check the battery / power connector (unprompted
   resets at 04:37:52 and 04:38:41). The bypass steers by the IMU heading.
2. **7 more bypass trials on the 04:32 image** (3/3 done), demo box 50-80 cm
   from the start so the line direction is learned: contact yes/no from the
   tester, the rest from the viewer. Closes FR-18, FR-19 and NFR-07's success
   rate; NFR-07's 7 s stays failed with the reason (§3, D14).
3. **NFR-08 during those trials:** tape-measure sensor-to-box at the stop in
   3-5 of them, and watch for a repeat of batch 1 trial 8 (contact with no
   stop in the log).
4. **Bench session:** layouts 2-8 (`tests/scan_bench/README.md` §1, 3 runs
   each) for FR-17, then mode 0 with the demo box for the NFR-06 range table.
   Both also finish NFR-16.
5. **Commit** the 3-4 Nov changes (obstacle, motion, tests, docs) and get
   Buddy 2's review of the motion part.
6. ~~LOOK-op check and `scan_logic_test.c` in `make -C tests` (NFR-17).~~ Done 4 Nov.
7. ~~Fill the NFR-20 placeholder~~ Wording in the NFR-20 row; NFR-06 wording
   proposed in its row.

## 5. Change after prof's feedback (3 Nov): dwell at each scan angle

Prof's comment: give each scan angle a buffer time so the measurement is
more accurate, instead of scanning many small angle steps.

| | Before | After (`obstacle_task.c`) |
|---|---|---|
| Coarse scan | 5 angles, settle 150 ms + **1 ping** each | 5 angles, settle 150 ms + **median of 3 pings** each (`look_dwell`) |
| Fine scan | **5° steps**, 1 ping each, median of 3 only at the edge, then step back inward if the median missed | **10° steps** (`FINE_STEP_DEG`), median of 3 at every angle, then **one 5° check** just outside the last hit |
| Scan budget | 5.5 s | **7.0 s** (`SCAN_BUDGET_MS`): each dwell costs ~0.13 s more, and NFR-07 can't be met anyway, so accuracy wins over scan time |
| Profile and decision (`scan_logic.c`) | | unchanged: the grid is still 5° slots, and unmeasured slots count as part of the object |

The edge is still found to 5°, but every reading behind it is now a median,
so one stray echo at the rim of the beam can't move the edge. The old
step-back-inward code was removed because a median at every angle makes it
unnecessary.

Status: builds clean (`SMP=1 CONSOLE=usb_cdc`), and the PC tests are
unaffected (they test `scan_logic.c`). **Not yet run on the car.** To check:
mode 3 with the demo box (§0) at 200 mm, 5 scans. Compare the width error
and the `scan took ... ms` figure with the old 4.6-5.0 s. Estimate:
~5-6 s *(calculated: servo ramp 10 ms/°, ~0.28 s dwell per angle)*.

**Run on the car 3 Nov:** mode 3, demo box at 200 mm: 150-162 mm wide
(0/5), scan 5.7-6.1 s, no `OVER`. See NFR-20 above.

## 6. Car tests and changes, 3-4 Nov 2026

| What | Result / change | Where |
|---|---|---|
| Width, 3 setups x 5 scans | NFR-20 row above; option 1 (keep k = 0.080) chosen | `tests/scan_bench/README.md` §2 |
| Stop distance | 230-246 mm at the first scan, 5 runs | NFR-08 row above |
| Full bypass and rejoin | Works once on the car; car passed ~30 mm closer than planned (side look 79 mm) | `logs/fr20_bypass_rejoin_2026-11-04.txt` |
| Boxed in by a second object | Car looped left/right. **Fixed:** sidestep lane must hold leg + D_stop + 50 mm (`sl_lane_clear`, Test 19), else LOOK 1 and motion tries the other side; Blocked after 3 bypass plans (`SL_MAX_BYPASS_PLANS`). **On the car:** the lane check fired at 02:44 and 04:37 (`short for leg + D_stop` → `trying BYPASS RIGHT`, then Blocked when both lanes were short). The retry limit has not been triggered on the car yet | `obstacle_task.c`, `scan_logic.c/.h` |
| Right encoder | ~1590 counts per turn vs 2535 left: distances ~20 % short, right wheel ~1.6x fast off the line. Scaled in motion (Buddy 2 to review) | `docs/motion/right_encoder_scaling.md` |
| IMU | Intermittent I2C: calibration reads x = y = z = 5987, false BUMP/tilt faults | wiring |
| Bypass margin | **Done:** `SL_PASS_MARGIN_MM` 80 (C_pass 248; the in-path corridor keeps `SL_MARGIN_MM` 50). The first side look then read 187-261 mm instead of 79-127 mm and the car no longer touched the box at the probe | `scan_logic.h` |
| Side switch | `switch_side()` waits 30 ms after STOP_SCAN so motion is Braked before the new BYPASS (the command was dropped before). **On the car 04:37:** motion took the new side at once (`bypass OUT` 10 ms after `trying BYPASS RIGHT`) | `obstacle_task.c` |
| Rejoin misses an angled line | Motion's STEP_IN also stops on a confirmed black (`dark_ms` < 30), not only on `rl` (Buddy 2) | `motor_task.c`, `docs/motion/right_encoder_scaling.md` §9 |
| Bypass rotated with the stop angle | Car stopped 20-30° off the line (line following wanders), so the encoder-only bypass missed the line on return. **Motion bypass now steers by the IMU heading** relative to the learned line direction, tracks the sideways distance and returns by it (Buddy 2) | §10 of the motion note; `logs/imu_bypass_left_right_2026-11-04.txt` |
| Counted trials, batch 1 (03:32 image) | 10 trials: 6/8 bypasses rejoined, contact in 4 (7, 8, 9, 10; on the turn in). Cause of 9-10: line direction averaged over 1.5 s (~7 cm) | `logs/trials_batch1_2026-11-04.txt` |
| Line direction over distance (04:32) | Averaged over 400 mm of travel, trusted after 300 mm; 80 mm margin past the estimate on the return. Sim: a slow ±20° wander learned to 0° | `motion_cfg.h` `MOTION_HD_*` |
| Batch 2 (04:32 image) | 3/3 real bypasses rejoined (box seen every time, 151-261 mm); 28.2-33.4 s. One stray detection (367 mm, width 0) after only 3 s of line, encoders-only, Blocked. **No contact in any of the 3 (tester).** IMU failed calibration again at 04:41; unprompted resets at 04:37:52, 04:38:41 (power?) | `logs/trials_batch2_2026-11-04.txt` |

