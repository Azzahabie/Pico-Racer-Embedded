# Week 10 check-in: what Buddy 4 can show

imu_task (tilt, hump detection and height, abnormal-motion safety stop,
`app_program/imu_task/`). Numbers come from the hand-push sessions of
26-28 Oct 2026 in `docs/imu/terrain_analysis_report.md`, the final-build
measurements in `docs/imu/week13_representation.md`, and the 1 Nov 2026
integration on the car (stage S6, `docs/INTEGRATION.md` §4, §5 and §8).

Shown on **master at `05f1fb7`** (PR #18, the integration baseline). The
console readability work on the `imu_task` branch (`d84590b`: turn
direction and count, lean, hump progress, mission summary) is not on master
yet and is not part of this check-in.

## 1. Against the requirements

| Requirement | Limit | What we have | Status |
|---|---|---|---|
| NFR-05 hump height | ±5 mm (target ±2 mm or 20 %) | Hand pushes: every clean pass within target (10 mm **3/3**, 12 mm **1/1**, 23.5 mm **3/3**); within the limit 10 mm **11/11**, 12 mm 2/3, 23.5 mm 3/5. **On the car (S6), prof's hump 195 × 10 mm: 9.7 and 9.6 mm.** | Met; 2 of 5 self-driven crossings done |
| FR-10 count each hump once | — | Flat-top, back-off and pick-up cases tested; 25 host scenarios incl. the prof's hump | Met |
| FR-12 highest hump per mission | — | In the `hump_state` word, reset on Ready → Running | Met; not yet in an E record (§4) |
| FR-28 abnormal tilt | T ≥ steepest hump + 5° | T = **25°**, N = 3 samples; steepest hump driven 15.7°. Knock filter: z must also drop below 950 mg | Met; one residual false fault (§5) |
| NFR-19 tilt → STOP_IMU | 40 ms | **20.5-20.8 ms** (6 trials); **21.4 ms** on the integrated car (S2) | Met |
| NFR-18 STOP_IMU → brake | 20 ms | **47-61 µs** (motion_task's own stamp) | Met |
| NFR-13 IMU fault | no data 50 ms | Calibration fail, abnormal tilt, read failure (50 ms) all fault; init failure not run | Met, one case untested |
| NFR-11 sample period | 10 ms ± 1 | Bench, Wi-Fi off: **9 653-10 352 µs** over 22 windows, worst work 1 466 µs. Driving (S4): 17/19 windows within, worst **11.7 ms** | **Open** while driving |
| FR-29 tilt and turn rate in telemetry | 1 s | `imu_ts` word read into `S.tlt`, `S.trn` every second | Built (from master) |
| FR-27 motion classes | — | still, turning, climbing, descending, impact, accelerating, moving | Built; thresholds untuned under motor power |

## 2. Other numbers worth showing

- **Footprint** (final build, `arm-none-eabi-size`): 9 430 B flash, 200 B
  static RAM, 4 KB stack; 1 task, 1 cyclic handler, 1 event flag,
  **0 mutexes**.
- **Shared words:** `hump_state` (4 B) and the tilt-state word `imu_ts`
  (4 B). Each has one writer and is stored whole every 10 ms; readers take
  one load, from either core, with no lock (D1-D3; the lock-free stress test
  passed 18/18 in S2).
- **Thresholds:** T = 25° (was 20°: two jerky pushes held > 20° for 3
  samples on a 15.7° hump); hump close after 40 mm level on the way down, or
  **120 mm level** in any case (was 250); minimum hump 4 mm.
- **Telemetry per IMU event:** 3 B queued, 9 B record E on the wire (packer
  ready, not sent yet).

## 3. Integration on the car (S6, 1 Nov)

Two crossings of the prof's hump (195 mm long, 10 mm high, peak halfway):

| Check | Result |
|---|---|
| Height (NFR-05) | **9.7 mm, 9.6 mm** |
| False IMU fault | none |
| False STOP_SCAN (scan debounces 4 readings on a hump) | none |
| Line lost on the hump | none (the V_hump cap and 2 × t_lost never fired) |
| Hump closing | **Too late**: stayed open ~250 mm past the hump |

The car's body is about as long as the hump, so it never reads the way
down and the hump only closed on the 250 mm "level" rule. Fixed in
`698d48b`: `HUMP_TOP_MAX_MM` 250 → 120, so hump mode should end about
13 cm after the crest instead of about 25 *(host test; not run on the car
yet)*.

Other changes to imu_task in the integration (on master):

- **`FAULT_IMU` on the shared fault flag.** `fault_set()` on a fault and
  on a failed calibration, `fault_clear()` only when a calibration passes.
- **Fault recovery.** `nav_cmd_mbf` is now bound to motion_task, so
  FAULT_CLEAR(IMU) after a good calibration clears STOP_IMU. Before, it
  stayed set until reboot.
- **Tilt-state word** `imu_ts.h` (FR-29): tilt 0-62°, 63 = not available
  (no good sample for 50 ms); turn rate ±2047 °/s, 0 = not available
  (`distance_state` silent for 50 ms).

## 4. Gaps to close (priority order)

1. **Finish S6:** 3 more self-driven crossings for the 5 planned, with the
   120 mm hump close on the car. Save the viewer logs.
2. **`E imu` record (M6).** imu_task is the only task not sending E records.
   `comm_w_pack_imu()` and the comms side exist; imu_task must queue an
   event on each class change / hump end.
3. **Boot glitch:** x = y = z = 5987 mg at some boots. Cause not found.
4. **Self-driven repeats.** NFR-05, flat-floor runs (no false humps) and the
   classifier thresholds (turn, still, impact 1.5 g) come from hand pushes.
   The `IMU_CSV` build is ready to log them.
5. **NFR-11 jitter while driving** (worst 11.7 ms; motion 11.5 ms too). Cause
   not found; shared with Buddy 2. Also repeat with Wi-Fi on.
6. **Readable console** (`imu_task` branch, `d84590b`): turn and lean
   events, hump progress and a mission summary. Built; to merge into master
   after its axis signs (`IMU_LEFT_SIGN` / `IMU_NOSE_SIGN`) are checked by
   tilting the car by hand.

## 5. Known limitations

1. **Wheel skid** is the main height error: distance is the mean of both
   wheels, so a stalled wheel reads low, a spinning one high.
2. **No gyroscope:** acceleration reads as tilt. Non-steady windows are
   gated, but hard pushes still disturb the estimate.
3. **Pivoting under-reads** (−3.4 mm on 10 mm, three times).
4. **A knock that also dips z can still trip T** (once, at 25.7°, while
   paused).
5. **Calibration accepts up to ±150 mg (~6°) off level.** Calibrate on flat
   floor, not on the ramp.
6. **Pausing on the slope** drops the unfinished hump (by design).
7. The magnetometer is read but unused.

## 6. Repeatable on the spot

- **Live:** the 1 s `[imu]` status line shows the class, tilt, speed, turn
  rate, hump state and height, and humps / last / max this mission. A
  `[imu] motion: <class>` line prints on every class change, and a timing
  line (sample period, NFR-11) every 10 status lines.
- **Raw data:** `IMU_CSV 1` (`imu_task.c`) prints one CSV row per 10 ms
  sample; `IMU_VERBOSE 1` adds the raw axes and z range.
- **Fault demo:** tilt the car past 25° during a mission: the
  `!!!!! IMU FAULT: abnormal tilt ... -> IMU_FAULT + STOP_IMU, x ms after the
  first sample` line gives the NFR-19 time. Recalibrate (GP20) to clear it.
- **PC tests** (no board): `imu_hump` (25 scenarios incl. the prof's hump)
  and `imu_safety`:

  ```sh
  docker compose run --rm build bash -lc "make -C tests"
  ```
