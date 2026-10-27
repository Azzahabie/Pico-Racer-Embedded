# Buddy 4 (imu_task) — everything to deliver, and today's plan

Team IS-24 · Pico Racer · µT-Kernel on RP2040 · GY-511 on I2C0 (SDA GP16, SCL GP17), accelerometer only, no gyro.
Sources: Project Briefing, Project Write-up, Week 6 design doc (embedded 23), Tribunal 1 Versions A/B/C (embedded 22), Buddy 2's follow-up thread (embedded 24), Change Requirement (minimum-bit), and the lecturer's ramp/Tribunal note.

---

## 1. What the brief and write-up ask of you

**Tasks:** IMU integration · sensor calibration · tilt detection · hump detection · peak hump measurement · motion classification · collision detection · turn-rate monitoring · motion event reporting.

**Motion events:** Stationary · Accelerating · Turning · Climbing hump · Descending hump · Sudden impact.

**Performance objectives:** reliable hump detection · accurate peak measurement · stable event classification.

**Deliverables**

| Deliverable | What it is in our repo |
|---|---|
| IMU processing module | `imu_task.c` (RTOS task, I2C, state machine) + `imu_logic.c` (plain C, no kernel/I2C calls, PC-testable) |
| Hump detection algorithm | Onset/end detection + height = Σ Δs·sin θ over the climb, in `imu_logic.c`, with PC tests |
| Terrain analysis report | Markdown in the repo: method, calibration, thresholds chosen from data, ramp test results, accuracy vs NFR-05 |

Mission requirements you own: #4 detect humps, #5 measure and report the **highest** hump peak of the run. Write-up also requires: µT-Kernel, C with the Pico C SDK, the course template repo, and **Barr-C** coding standard.

---

## 2. Changed requirements (apply these, don't compare)

**Ramp change.** The final ramp is about the height of the **lower practice ramp** (slightly lower or slightly higher). Ignore the higher ramp.
- Tune θ_on, T and the correction factor on the lower ramp only.
- Test at the lower ramp height and at least one slightly lower and one slightly higher setup (shim it with a book or card) so you show it generalises.
- Smaller hump means smaller height, so NFR-05's 0.5 cm floor and sub-mm accumulator precision matter more.

**Tribunal 1 comparison dropped.** You do **not** need to compare original vs revised design. Implement the additional requirement (minimum-bit) and give evidence for the **final** implementation only. The "unpacked baseline" and "before/after" plans from Tribunal 1 become optional. (The Change Requirement's Week 13 list still names RAM and message-size "comparison"; by the lecturer's note, report the final numbers, and add a baseline only if the team decides to.)

**Minimum-bit change requirement (Week 13), your part:**
- Each field uses only the bits its range needs, with masks and shifts (not compiler bit-fields) for anything on a queue or MQTT.
- Your fields: `hump_state` 2 bits (NONE/CLIMBING/DESCENDING), motion event class 3 bits (6 events incl. Accelerating fits in 3), IMU_FAULT = bit 2 of the shared fault mask, tilt, highest peak, hump record fields, update-counter check on `distance_state`.
- Packed data shared across tasks must be race-free: one writer per word, one aligned 32-bit store/load, or a mutex.
- MQTT: compact binary. Buddy 1 packs it with `comm_w_pack_<kind>()` and `comms_tele_send()`; you send Buddy 1 each field's **name, range, unit** (MQTT_SCHEMA.md §5.2).
- Week 13 evidence: your rows in the **data representation table** (field, range, bits, representation), RAM and message size of your records, and proof that NFR-05, NFR-18, NFR-19 and NFR-11 still pass on the final packed build.

---

## 3. Week 6 requirements you own (pass/fail list)

| ID | Requirement (short) | How it's verified |
|---|---|---|
| FR-02 | No mission until IMU calibrated (and line sensors) | Try to start before/after calibration |
| FR-10 | Detect each hump with IMU data | Ramp run records a hump event |
| FR-11 | Estimate each hump's peak height with the documented method | Ramp of known height gives an estimate |
| FR-12 | Keep the **greatest** peak of the mission, available for telemetry | Several humps; check reported max |
| FR-23 | Telemetry includes hump measurements, motion events, tilt, turn rate | Inspect published messages |
| FR-27 | Classify stationary, turning, climbing, descending, sudden impact; report each change | Stop/start/turn/hump/bump tests |
| FR-28 | Abnormal motion = tilt > T for N = 3 samples; a normal hump must **not** trigger; T ≥ steepest hump pitch + 5° | Humps at several speeds (no fault); tilt past T (fault) |
| FR-29 | Tilt (accel) and turn rate (encoders) in status telemetry ≥ 1 Hz | Known tilt angle and known turn rate |
| NFR-05 | Peak height within **0.5 cm or 20%** (whichever larger), 5 traversals per reference hump | Ramp runs vs measured height |
| NFR-11 | imu_task runs every **10 ms ± 1 ms** | Log task start times |
| NFR-13 | IMU fault = init fail, cal fail, abnormal motion, or no valid sample within **Y = 50 ms**. Before a mission: block movement, go to Fault. During a mission: urgent stop + report | Simulate each fault separately (incl. unplugging the IMU) |
| NFR-18 | motion_task brakes within 20 ms of STOP_IMU (Buddy 2's side, your trigger) | Time request → stop |
| NFR-19 | STOP_IMU raised within **40 ms** of first sample above T | Tilt the car, log 1 MHz timestamps |

**Proposed thresholds to replace with measured values:** T = 20°, N = 3, A = 1.5 g (impact), θ_on = 5° for 3 samples, θ_off = 3° of flat for 3 samples, Y = 50 ms, V_hump = 15 cm/s, stationary = no dist change and |turn| < 5 °/s for 5 samples, turning = |turn| > 20 °/s.

**Assumptions to close:** A4 (IMU tells humps from normal motion, T above steepest hump, height method works) and A8 (encoder turn rate good enough for FR-27/29, with Buddy 2).

### Statechart 6.4 (must match the code)
Initialising → Awaiting Calibration → Calibrating → Ready → Monitoring ⇄ Measuring Hump (Climbing/Descending), and Fault.
- **Initialising:** set up the IMU on I2C0 GP16/17, check readings. Fail → report IMU fault → Fault.
- **Calibrating:** car still; record flat reading on all 3 axes (sets up/forward/sideways mapping, board is on its edge). OK → CAL_OK; fail → CAL_FAIL + Fault.
- **Ready:** on Ready→Running reset the mission max; on Paused→Running keep it.
- **Monitoring:** sample every 10 ms, classify events, report tilt + turn rate with the 1 s status, watch for onset, abnormal tilt, read failure.
- **Measuring Hump:** pitch × encoder Δs builds height; on end, record peak, update mission max, send telemetry. Pause/end/fault discards the unfinished hump. Write `hump_state` on entering Climbing/Descending; NONE on exit, pause, end, fault.
- **Fault:** report to nav_ctrl_task; abnormal motion or read failure during a mission also sets **STOP_IMU** on `motion_flg`. Leave only via a new calibration request. STOP_IMU stays until calibration succeeds and nav_ctrl_task sends FAULT_CLEAR(IMU).
- **Method details:** low-pass filtered accel pitch; Σ Δs·sin θ over the climb using wrap-safe `dist_mm` deltas; ignore negative Δs (reversing); **hold the last pitch while distance_state says ACCELERATING/DECELERATING**; correction factor from reference runs; fallback distance = commanded speed × time.

---

## 4. Tribunal 1 commitments (pre, live, post)

**Version A (pre-Zoom), your position:** pitch × encoder distance with correction factor; tilt above T is the only stop trigger; impacts are events only; accelerometer-only pitch; turn rate from encoders.

**Version B (live) decisions with your name on them:**
- `hump_state` → 2-bit field; motion event class → 3-bit field; masks/shifts, not bit-fields, for queue/MQTT data.
- imu_task stays priority 12, core 1, 10 ms. **Measure tick interval and WCET** (on the final build).
- Height method unchanged; `dist_mm` stays in mm; the height accumulator keeps **sub-mm fixed-point** (≈1.5 mm Δs and ≈0.13 mm height per sample at 15 cm/s, 5°).
- Constraint response: safety and accuracy before bits; STOP_IMU stays its own event-flag bit; leave a field unpacked and document why if packing hurts timing/accuracy.
- Interface map: `imu_logic.c` (pure C, PC-tested by replaying logs), `hump_state` interface, STOP_IMU + IMU_FAULT/CAL_OK/CAL_FAIL on `nav_flg`.
- Test plan "IMU stop path" (owner you): tilt past T for 3 samples with PID running and Wi-Fi streaming; STOP_IMU ≤ 40 ms, motor stop ≤ 20 ms, **worst of 100 trials**; serial log with timestamps + commit link. (Per the lecturer's note: final build only, no baseline needed.)
- Two-core fault-bit stress test (Buddies 1, 2, 4, 5): your task sets/clears bit 2 repeatedly; no bit lost.

**Version C (post) items you must honour:**
- Row 3: shared fault flag `fault_flg` owned by nav_ctrl_task; **you set/clear only bit 2 IMU_FAULT** via `fault_set()` / `fault_clear()`; it clears when recalibration succeeds; IMU_FAULT on `nav_flg` stays the wake event.
- Row 17: `distance_state` is one 32-bit word, no mutex: bits 0–14 `dist_mm` (wrapping, take wrap-safe deltas), 15–16 STEADY/ACCEL/DECEL, 17–19 update counter (detect skipped/repeated reads), 20–31 signed turn rate in °/s, **counter-clockwise (left) positive**. Stationary check compares `dist_mm`, not the whole word.
- Row 10: A4 open; don't present thresholds as validated until measured.
- From the follow-up thread: Buddy 2 agreed `hump_state` is **one word, no mutex**, turn-rate averaging ≤ 50 ms (5 ticks), and will share the A8 turn CSV (1 MHz timestamp, wheel speeds, turn rate; 90° turn at 3 speeds × 5, plus a straight run). You agreed to log magnetometer heading only as a slip cross-check.

**Two document fixes to make while you're there:**
1. Version C row 2 still says "hump_state keeps its mutex". The agreed design is one 32-bit word, single writer, readers on core 1. Update the row to match the code.
2. The docs say LSM303**DLHC**; your flat readings (0.244 mg/LSB) match the LSM303**D** path. Make the docs say which chip the code drives.

---

## 5. Gaps in the current code (from the earlier review)

- No re-setup after the IMU chip resets (read fails after a brown-out or touch). Needs a re-init path → otherwise Fault.
- Nothing sets `NAV_FLG_CAL_OK_IMU`, `NAV_FLG_CAL_FAIL` or `NAV_FLG_IMU_FAULT` (defined in `nav_ctrl.h`).
- STOP_IMU, `fault_set/clear(IMU_FAULT)`, `hump_state` word, telemetry records not wired yet.
- Encoder distance comes from `motion_ds_read()` / `motion_ds_delta_mm()` (core 1 only; imu_task is on core 1).
- Make sure imu_task is created with **TA_ASSPRC, core 1, priority 12** (Buddy 1's merge note).
- Electrical: "read failed" when touching the pins. Secure the wiring before car tests; it will also show up in your read-failure fault test.

---

## 6. Today's checklist

| Step | What | Needs the car? | Needs a teammate? |
|---|---|---|---|
| 0.1 | Pull latest master into your branch; build | No | No |
| 0.2 | imu_task pinned (TA_ASSPRC, core 1, prio 12); I2C0 on GP16/17, LED off GP16 | No | Buddy 1 / Buddy 2 PR |
| 1.1 | `imu_logic.c/.h`: calibration, axis mapping, filtered pitch, tilt | No | No |
| 1.2 | Hump algorithm: onset/end, Σ Δs·sin θ (fixed-point), pitch hold while accelerating, peak + mission max | No | Buddy 2's `distance_state` layout |
| 1.3 | Motion classifier (6 events), impact > A, abnormal tilt (N of T), read-fail timer Y | No | No |
| 1.4 | PC unit tests + CSV replay harness | No | No |
| 2.1 | `imu_task.c` state machine 6.4, incl. re-init after chip reset | No | No |
| 2.2 | Flags: CAL_OK/CAL_FAIL/IMU_FAULT on `nav_flg`, STOP_IMU on `motion_flg`, `fault_set/clear` bit 2 | No (bench to test) | Buddy 5's fault flag (stub if missing) |
| 2.3 | `hump_state` word + packed telemetry records; send field list to Buddy 1 | No | Buddy 1's schema |
| 2.4 | CSV logging mode over USB (1 MHz timestamp, raw axes, pitch, dist, state) | No | No |
| 3.1 | Bench: noise motors off/on, tilt by known angle, calibration | **Yes (wheels raised)** | No |
| 3.2 | Flat runs: accelerate, brake, turn, stop → no false humps | **Yes** | Buddy 2 motion |
| 3.3 | Lower-ramp reference runs: 3 speeds × 5, plus slightly lower/higher | **Yes** | Buddy 2 motion |
| 3.4 | Set θ_on, θ_off, T, A, correction factor from data; re-run 5 traversals for NFR-05 | **Yes** | No |
| 3.5 | Fault tests: init fail, cal fail, tilt past T, unplug IMU mid-run (NFR-13) | **Yes** | Buddy 2 (STOP_IMU), Buddy 5 (Fault) |
| 3.6 | Timing: NFR-19 (≤40 ms), NFR-18 hand-off, NFR-11 tick + WCET, with Wi-Fi on | **Yes** | Buddies 1 and 2 |
| 3.7 | A8 turn-rate check against a timed 90° turn | **Yes** | Buddy 2 |
| 4.1 | Terrain analysis report | No | No |
| 4.2 | Week 13 data-representation rows + RAM/message size of your records | No | No |
| 4.3 | Fix the two doc mismatches (§4); Barr-C pass on your files; open PR | No | Team review |

### Step by step (with what to type into Claude Code)

**Morning: code without the car**
- [ ] **0.1** Pull master and build. *Prompt:* "Merge latest master into my branch, resolve app_main.c conflicts keeping my imu_task creation, and build with the usual make command."
- [ ] **0.2** *Prompt:* "Check imu_task is created with TA_ASSPRC on core 1 at priority 12, and that I2C0 is muxed to GP16/GP17 with nothing else using GP16. Show me what needs to change before editing."
- [ ] **1.1** *Prompt:* "Create imu_logic.c/.h as plain C with no kernel or I2C calls: calibration from N still samples (store flat vector, find up/forward axes), low-pass filtered pitch in fixed-point, tilt angle. Follow Barr-C."
- [ ] **1.2** *Prompt:* "Add the hump algorithm to imu_logic.c from Statechart 6.4: onset when pitch > θ_on for 3 samples, end when within θ_off of flat for 3 samples, height += Δs·sin θ with Δs from wrap-safe 15-bit dist_mm deltas, ignore negative Δs, hold pitch while distance_state says ACCEL/DECEL, sub-mm fixed-point accumulator, record peak and mission max. Make thresholds #defines."
- [ ] **1.3** *Prompt:* "Add the motion classifier: Stationary (no dist change and |turn| < 5 °/s for 5 samples), Accelerating (from distance_state bits), Turning (|turn| > 20 °/s), Climbing/Descending (from hump phase), Sudden impact (|a| > A). Add abnormal tilt (> T for N samples) and a read-failure timer (no valid sample in Y ms). Output a 3-bit event code."
- [ ] **1.4** *Prompt:* "Write PC tests for imu_logic: flat run gives no hump, tilt past T for 3 samples sets the abnormal flag, a synthetic 2 cm ramp gives a height within NFR-05, dist_mm wrap is handled, reversing is ignored. Add a CSV replay tool. Run them."
- [ ] **2.1** *Prompt:* "Rewrite imu_task.c as the Statechart 6.4 state machine using imu_logic. If a read fails, retry setup once; if it still fails within 50 ms, go to Fault."
- [ ] **2.2** *Prompt:* "Set NAV_FLG_CAL_OK_IMU / NAV_FLG_CAL_FAIL / NAV_FLG_IMU_FAULT on nav_flg, set STOP_IMU on motion_flg for abnormal tilt or read failure during Running, and call fault_set/fault_clear for bit 2. Stub fault_set if Buddy 5's helper isn't on master yet."
- [ ] **2.3** *Prompt:* "Publish hump_state as one aligned 32-bit word (2-bit state). Add packed telemetry for hump record, motion event, tilt, turn rate and highest peak using Buddy 1's comm_w_pack helpers, and draft the field/range/unit list I send to Buddy 1."
- [ ] **2.4** *Prompt:* "Add a compile-time IMU_LOG build that prints CSV each tick: t_us, ax, ay, az, pitch, dist_mm, turn, state, height."

**Afternoon: on the car** (secure the IMU wiring first; hold RESET to stop the motors if needed)
- [ ] **3.1** 🚗 Wheels raised: log 10 s still with motors off, then on → noise. Tilt nose-up on a known angle (phone level / protractor) → confirm axes and pitch sign.
- [ ] **3.2** 🚗 Flat runs: start, stop, brake, turn left/right. Expect zero humps, correct events, no fault.
- [ ] **3.3** 🚗 Measure the lower ramp's true height with a ruler. Run 3 speeds × 5 traversals; save every CSV. Repeat with the ramp shimmed slightly lower and slightly higher.
- [ ] **3.4** 🚗 *Prompt:* "Here are my ramp CSVs. Replay them through imu_logic, propose θ_on, θ_off, T (≥ steepest pitch + 5°), A and a correction factor, and show error per run against NFR-05." Then update the #defines and do 5 fresh traversals to confirm.
- [ ] **3.5** 🚗 Fault tests, one at a time: IMU unplugged at boot (init fail), car moved during calibration (cal fail), tilt by hand past T while driving (STOP_IMU), pull SDA mid-run (read fail). Film each; check the car brakes and the system shows Fault, and FAULT_CLEAR only after recalibration.
- [ ] **3.6** 🚗 With Wi-Fi streaming: tilt past T and log 1 MHz timestamps first-sample → STOP_IMU (≤ 40 ms) and → brake (≤ 20 ms, Buddy 2's side). Log tick intervals and imu_task WCET. Aim for 100 trials; if time runs out, say how many in the report.
- [ ] **3.7** 🚗 With Buddy 2: timed 90° turns, compare reported turn rate; straight run should show no Turning events.

**Evening: write it up**
- [ ] **4.1** *Prompt:* "Write docs/terrain_analysis_report.md from my CSVs: setup, calibration, axis mapping, method (Σ Δs·sin θ, pitch hold, correction factor), threshold choices with plots/tables, ramp results vs NFR-05 including the lower/higher shims, event classification results, fault and timing results, limitations (no gyro, slip, magnetometer not used)."
- [ ] **4.2** *Prompt:* "Add my rows to the Week 13 data representation table (field, range, bits, representation) and report the RAM and message size of my records from the linker map."
- [ ] **4.3** Update Version C row 2 (hump_state word, no mutex) and the chip name; replace every "proposed" IMU value with the measured one. *Prompt:* "Do a Barr-C review of imu_task.c and imu_logic.c and fix issues." Commit, push, open a PR for team review.

**If you run out of time**, the order that protects your marks: 1.2 → 2.1/2.2 → 3.3/3.4 (NFR-05) → 3.5 (safety) → 4.1 report → 3.6 timing → the rest.
