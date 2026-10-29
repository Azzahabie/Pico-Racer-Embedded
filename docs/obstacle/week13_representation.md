# Week 13 evidence: minimum-bit data representation (Buddy 5)

scan_task (obstacle scanning and avoidance, `app_program/obstacle_task/`) and
nav_ctrl_task (mission state, buttons, shared fault flag,
`app_program/nav_task/`). Covers the four items the Change Requirement asks
for: the data representation table, RAM before and after, message sizes
before and after, and correctness and performance.

**Before** is commit `c3dea9d` ("before requirement change for obstacle
task"). **After** is the tree this file was committed with. Legend: **M** =
measured with the car's ARM toolchain (`arm-none-eabi-size`, `nm -S`) by
`tests/week13/scan_measure.sh`; **C** = calculated from the code. Re-run:

```sh
sh tests/week13/scan_measure.sh          # builds c3dea9d and HEAD in Docker
```

## 1. Data representation table

### Data that leaves scan_task or nav_ctrl (shared)

| # | Field | Writer -> reader(s), core | Valid range | Bits | Representation and synchronisation |
|---|---|---|---|---|---|
| 1 | Avoid command | scan_task (core 1) -> motion_task (core 1), `avoid_cmd_mbf` | op 1 reverse / 2 bypass; side left/right; distance 1-511 mm | 3 + 1 + 9 = **13** (3 spare) | one `uint16_t`: bits 0-2 op, bit 3 side, bits 4-12 mm (`motion_avoid_cmd_pack`, `messages.h`). Message buffer copies it: no shared memory, no race. Was a 4-byte struct of whole bytes. |
| 2 | Obstacle telemetry (event kind 4) | scan_task -> comms_task (core 0), `tele_mbf` | width 0-255 cm (clamped), bypass 0 none / 1 left / 2 right | 3 kind + 8 + 2 = **13** | 2 bytes, `comm_w_pack_obstacle` (Buddy 1's schema v5); 9-byte record E on MQTT. Was a 16-byte struct with a whole byte or 16-bit word per field (`scan_tele_t`), larger than `tele_mbf`'s 7-byte maximum message. |
| 3 | Shared fault bitmask | each owner sets / clears its own bit; read by nav_ctrl and comms_task (both cores) | motor stall, encoder, IMU, no echo (sensor lost), line sensor, Wi-Fi loss | **1 each, 6 total** | one uT-Kernel event flag (`fault_flg`, `fault_set` / `fault_clear` / `fault_bits`); bits are `FAULT_*` in `messages.h`. `tk_set_flg` / `tk_clr_flg` run under the kernel's cross-core lock, so two cores never lose each other's bits (the M0+ has no atomic read-modify-write). Copied into record S `faults` (6 bits). Before: no shared mask; sensor lost was its own 16-byte `'F'` record. |
| 4 | STOP_SCAN, CMD | scan_task -> motion_task | set / clear | 1 each | bits 2 and 1 of `motion_flg` (event flag, atomic kernel calls). Kept as its own bit, not packed with data (Tribunal: safety before bits). |
| 5 | REVERSE_DONE, LINE_FOUND, LINE_LOST, TICK, ECHO | motion_task / cyclic handler / ECHO_ISR -> scan_task | set | 1 each | bits of `scan_flg` (event flag) |
| 6 | BLOCKED and the other nav_flg events | scan_task, imu_task, motion_task, buttons -> nav_ctrl | set | 1 each | bits of `nav_flg` (event flag) |
| 7 | Mission state | nav_ctrl (core 0) -> scan_task, imu_task, motion_task (both cores) | 7 states | 3 | one aligned 32-bit word, written only by nav_ctrl with one store, read with one load: whole-word access is atomic on the M0+, so no mutex is needed (config.h allows only 4). 29 bits unused by choice: packing it into a shared word with other fields would need a read-modify-write across cores. |
| 8 | Hump active (read) | imu_task's `hump_state` word -> scan_task | 0 none / 1 climbing / 2 descending | 2 | read through Buddy 4's packed word, one load (`imu_word_read`, `IMU_W_GET(w, HUMP)`); see `docs/imu/week13_representation.md` |
| 9 | Echo edge timestamps and flags | ECHO_ISR -> scan_task (core 1) | 32-bit microsecond timer; seen 0/1 | 32 + 32; 1 + 1 | `g_rise_us`, `g_fall_us` stay 32-bit (wrap-safe differences of the 1 MHz timer); `g_rise_seen`, `g_fall_seen` one byte each, written whole by the ISR. Not packed into one byte, because the ISR and the task would then both read-modify-write it. |

### Data private to scan_task (one task, no lock needed)

| # | Field | Valid range | Bits | Representation |
|---|---|---|---|---|
| 10 | Scan state (Statechart 5 + Reversing) | 9 states | **4** | bits 0-3 of `scan_ctx_t.bits` (masks and shifts, `CTX_STATE_MASK`) |
| 11 | reverse tried, after reverse, BLOCKED sent, abort scan, lost in scan, command warned, bench mode | 0/1 each | **1 each** | bits 4-8, 11, 12 of `bits` |
| 12 | No-echo pings in a row, in a scan | 0-3 (saturating) | **2** | bits 9-10 of `bits` |
| 13 | Forward-ping counters: close, clear, no-echo, echo | 0-7, 0-3, 0-3, 0-3 | 3 + 2 + 2 + 2 = **9** | `sl_runs_t.packed`, one `uint16_t` (was 4 bytes); PC Test 14 checks saturation and that nothing spills out of 9 bits |
| 14 | Chosen action | 0-4 | 3 | `uint8_t` (smallest addressable unit; was a 4-byte enum) |
| 15 | Fine-scan start edges, last servo angle | -45..+45 deg, or 127 = none | 7 | `int8_t` each (was `INT`, 4 bytes) |
| 16 | Servo angle (servo.c) | -45..+45 deg | 7 | `int8_t` (was `INT`) |
| 17 | Scan grid, one slot per 5 deg | 0-4000 mm, 0 = far, 0xFFFF = not measured | 12 (+2 codes) | `uint16_t` per slot. Kept at 16 bits by the Tribunal 1 decision: 12-bit packing saves 9 bytes and costs a shift and mask on every ping. 19 slots (-45..+45; was 25 slots for -60..+60, a scan-range change, not a representation change). |
| 18 | Slots with no echo this scan | 19 slots | 19 | `uint32_t` bitmask (smallest unit holding 19 bits) |
| 19 | Obstacle profile (FR-16): closest point, angles, edges, width, offsets, clearances | -4000..+9999 mm, -45..+45 deg | 15 | `int16_t` each (was `int32_t`); arithmetic done in `int32_t`, stored with an explicit cast; `_Static_assert` on the largest value. 4 flags as `bool`. |
| 20 | State entry time, scan start time | 32-bit microsecond timer | 32 | `UW`: wrap-safe differences need the full width |

## 2. RAM usage, before and after (M)

### scan_task context (`scan_ctx_t`, on scan_task's stack)

| | Before | After | Change |
|---|---|---|---|
| `scan_ctx_t` | 136 B | **84 B** | **-52 B (-38 %)** |
| of which `sl_profile_t` | 52 B | 26 B | -26 B (-50 %) |
| of which `sl_runs_t` | 4 B | 2 B | -2 B (-50 %) |
| of which the grid | 50 B (25 slots) | 38 B (19 slots) | -12 B, from the narrower scan range |

The context lives on scan_task's fixed 4096-byte stack, so it lowers the
stack in use, not the image's static RAM.

### Static RAM (`.data` + `.bss`) of scan_task and nav_ctrl objects

| Object | Before | After | Why |
|---|---|---|---|
| scan_if.o | 32 B | 24 B | tele_mbf ID and private drop counter gone (comms counts drops) |
| servo.o | 0 B | 1 B | servo angle now remembered (ramped moves), as `int8_t` |
| ultrasonic.o | 28 B | 28 B | unchanged (row 9) |
| nav_ctrl.o | 28 B | 32 B | +4 B: the shared fault flag's ID (row 3) |
| **Total** | **88 B** | **85 B** | -3 B while adding the shared fault flag and the servo ramp |

Whole image, for context: `.data` 172 -> 176 B, `.bss` 19 876 -> 31 564 B.
The growth is the other subsystems merged since `c3dea9d` (IMU, line and
barcode, motion, comms), not this change; Buddy 5's share is in the table
above.

## 3. Message sizes, before and after (M)

| Message | Path | Before | After | Change |
|---|---|---|---|---|
| Avoid command | scan_task -> motion_task, `avoid_cmd_mbf` | 4 B payload, 8 B buffer slot | **2 B** payload, 8 B slot | -50 % payload; the slot is the kernel's 4-byte header + 4-byte rounding either way |
| Obstacle profile | scan_task -> comms_task, `tele_mbf` | 16 B `scan_tele_t`, 20 B slot | **2 B** body, 8 B slot | -88 % payload, -60 % slot |
| Obstacle profile on MQTT | comms_task -> broker | (did not fit the intake: 16 B > 7 B max) | **9 B** record E | binary, 53-bit header + 13-bit body |
| Sensor lost | scan_task -> telemetry | 16 B `'F'` record | **1 bit** in the shared fault flag, carried in record S | |

## 4. Correctness and performance

**Correctness (no lost function):**
- PC tests, `tests/scan_logic_test.c`: 15 test groups, all pass on the final
  representation, including Test 14 (packed counters saturate and stay in 9
  bits) and Test 15 (fine-scan start between two objects).
- Firmware builds clean (0 warnings) in the car profile and the Wi-Fi/MQTT
  profile; `scan_logic.c` is also clean under `-Wconversion`.
- `messages.h` packers: `tests/host/test_messages.c`, 229 checks pass.
- Bench, 29 Oct (`tests/scan_bench/README.md`): detection, coarse and fine
  scans, the action, Blocked, mission pause after t_block, and the
  sensor-lost fault bit all behaved as designed on the packed context.

**Performance (no unacceptable timing change):**
- Urgent stop (NFR-18): STOP_SCAN to brake write 25-38 us in every bench
  run, against 20 ms. STOP_SCAN is still its own event-flag bit, so the
  packing is not on the stop path.
- Scan time (NFR-07 budget): 4.6-5.0 s per scan, under the 5.5 s scan
  budget. A scan is dominated by servo moves and 150 ms settles (about
  0.15-0.2 s per angle); the extra shifts and masks cost microseconds.
- motion_task timing with scan_task running: tick 10 ms +/- 0.2, 0 lost
  ticks, watchdog gap <= 10.4 ms (`[motion] timing` lines in the bench logs).

**Open:** the `int16_t` profile and the `int8_t` angles (rows 15, 16, 19)
were shrunk after the bench session; the PC tests cover them, and one
mode-4 scan on the car should be repeated to close this. Timing with Wi-Fi
streaming (NFR-11) is not yet measured.
