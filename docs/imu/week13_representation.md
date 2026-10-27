# Week 13 evidence: data representation table (IMU, Buddy 4)

One row per piece of application data imu_task owns, writes or reads, with its
valid range, the bits it needs and the form it has in the final build. By the
lecturer's note on the changed requirement, this reports the **final**
implementation only; no Week 6 baseline was built.

Legend: M = measured, C = calculated from the code. Sizes measured on the car
build of commit `1081a5f` (`arm-none-eabi-size -B` and `nm -S` on the
`imu_task` objects, 2026-10-28).

## Data that leaves imu_task

| # | Field | Writer -> reader(s), core | Valid range | Bits | Final form and sync | Size | Status |
|---|---|---|---|---|---|---|---|
| 1 | `hump_state` word (rows 2-8) | imu_task (core 1) -> any task on core 1 | — | 32 | one aligned `UW`, rebuilt in a local and stored once per 10 ms; one load to read; no mutex (Tribunal Version C row 2: one writer, one word). Masks and shifts, not bit-fields (`imu_task.h`: `IMU_W_*`, `imu_w_pack`, `IMU_W_GET`) | 4 B RAM (M, nm: `imu_word`) | built |
| 2 | hump state | as row 1 | 0 none, 1 climbing, 2 descending (3 unused) | 2 | bits 0-1 | in row 1 | built |
| 3 | motion event class (FR-27) | as row 1 | 0 still, 1 turning, 2 climbing, 3 descending, 4 impact, 5 accelerating, 6 moving | 3 | bits 2-4 | in row 1 | built |
| 4 | hump height now | as row 1 | 0-255 mm, clamped (reference humps 10-24 mm, M) | 8 | bits 5-12, whole mm | in row 1 | built |
| 5 | hump count, this mission | as row 1 | 0-15, clamped | 4 | bits 13-16 | in row 1 | built |
| 6 | highest hump, this mission (FR-12) | as row 1 | 0-255 mm, clamped | 8 | bits 17-24; reset on Ready -> Running | in row 1 | built |
| 7 | IMU fault | as row 1 | 0/1 | 1 | bit 25 | in row 1 | built |
| 8 | tilt | as row 1 | 0-63 deg, clamped (abnormal above 25, M) | 6 | bits 26-31, whole degrees | in row 1 | built |
| 9 | STOP_IMU | imu_task -> motion_task (core 1) | set / never cleared by imu_task | 1 | its own bit (bit 3) on `motion_flg`; not packed with anything, by the Tribunal's "safety before bits" decision | 0 B (event flag) | built; brake 47-61 us after (M) |
| 10 | CAL_OK_IMU, CAL_FAIL, IMU_FAULT | imu_task (core 1) -> nav_ctrl_task (core 0) | set | 1 each | bits 0x0008, 0x0010, 0x0020 of `nav_flg` | 0 B | built |
| 11 | CAL_REQ | nav_ctrl_task -> imu_task | set, cleared on read | 1 | bit 0 of `imu_flg` [PROPOSED, agree with Buddies 3, 5] | 0 B | built |
| 12 | FAULT_IMU in the shared fault mask | imu_task -> all | 0/1 | 1 | bit 2 of `fault_flg` (`messages.h`) | 0 B | waits for Buddy 5's `fault_set()` |
| 13 | Telemetry event, kind 3 imu | imu_task -> comms_task (core 0) via `tele_mbf` | class 0-4 (schema), count 0-15, height 0-255 mm | 3 + 3 + 4 + 8 = 18 | `comm_w_pack_imu()` (Buddy 1, `messages.h`) | 3 B in the queue, 9 B record E on the wire (C, `COMM_W_*_SIZE_IMU`) | packer exists; imu_task does not send yet (Buddy 1, plan 2.3). Schema class range is 0-4; ours uses 0-6 |
| 14 | Status S: `hum`, `hgt` | comms_task's fold of row 13 | 0-15; 0-255 mm | 4 + 8 | fields of record S | in Buddy 1's 16 B S record | as row 13 |

## Data imu_task reads

| # | Field | Writer -> reader | Valid range | Bits | Form | Notes |
|---|---|---|---|---|---|---|
| 15 | `distance_state` (dist_mm, state, counter, turn) | motion_task -> imu_task, both core 1 | Buddy 2's rows 6-9 | 15 + 2 + 3 + 12 | one `UW`, one load (`motion_ds_read`) | wrap-safe mm deltas each 10 ms sample and each 100 ms hump window; counter used to count publishes |
| 16 | mission state | nav_ctrl_task -> readers | 7 states | — | one aligned 32-bit store (Buddy 5) | read every sample: fault scope, mission reset |

## Kept unpacked on purpose (private to imu_task)

| # | Field | Range | Form | Why not fewer bits |
|---|---|---|---|---|
| 17 | height accumulator `h_um`, peak, trough | ±2^31 µm | `W` (32 bit), µm | sub-mm fixed point for NFR-05 (Tribunal Version B: one window adds ~1.3 mm at 15 cm/s, 5°); accuracy before bits |
| 18 | accel sample x, y, z | ±8000 mg | `W` locals on the stack | sums of 10-100 samples would overflow 16 bits |
| 19 | calibration offsets `acc_ofs` | ±200 mg | 3 x `W`, 12 B | could be 3 x `H` (6 B); left as `W` to match the arithmetic. 6 B saving noted |
| 20 | hump detector state (`hump.c`) | — | 26 variables, 124 B | single-task working state, never shared; readability of the algorithm over 100 B |
| 21 | classifier / safety state | — | 28 B + 12 B | as row 20 |

## Totals (final build)

| | Measured |
|---|---|
| Code (flash), all imu_task objects | 9 430 B (`hump.o` 1 947, `imu_class.o` 423, `imu_safety.o` 192, `imu_task.o` 6 868) |
| Static RAM (data + bss) | 200 B (16 + 184) |
| Stack | 4 096 B allocated (`STACK_SZ`, shared constant); high-water mark not measured yet |
| Kernel objects | 1 task, 1 cyclic handler, 1 event flag; **0 of the 4 mutexes** |
| Shared word | 4 B (`hump_state` word) |
| Telemetry per IMU event | 3 B queued, 9 B on the wire |

## Requirements still met on this build

| Requirement | Measured on the final build |
|---|---|
| NFR-05 | 10 mm ramp 8/8; 23.5 mm 3/3 clean (terrain_analysis_report.md §4) |
| NFR-11 | sample period 9 653-10 352 µs over 22 windows of 10 s; worst work 1 466 µs |
| NFR-19 | 20.5-20.8 ms, 4 trials (limit 40 ms) |
| NFR-18 | brake 47-61 µs after STOP_IMU (motion's stamp) |
