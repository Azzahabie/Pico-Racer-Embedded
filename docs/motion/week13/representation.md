# Week 13 evidence 1: data representation table (motion, Buddy 2)

One row per piece of application data motion owns, writes or reads, with
its valid range, the bits it needs, and its form before (Week 6) and after
the minimum-bit change. Skeleton: rows marked "not built" are filled in as
each feature lands (plan Stage 1), using the same `MOTION_PACKED` switch.

Legend: M = measured, E = estimated, C = calculated. "switch" = compared by
building with `MOTION_PACKED` 0 and 1 (`tests/week13/measure.sh`); "no" =
the same in both builds (no Week 6 form, owned by someone else, or kept
unpacked on purpose, with the reason). Method and numbers:
[baseline.md](baseline.md).

| # | Field | Owner; switched? | Writer -> reader(s), core | Valid range (M/E) | Bits | Before (Week 6 form, size, sync) | After (form, size, sync) | RAM before -> after | Message bytes before -> after | Status today |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Encoder count, left and right | Buddy 2; switch | ENC_ISR -> motion_task, both core 1 | 0..65535, wraps; 34 per tick at 350 per-mille (M, 24 Oct); wraps only above 32767 per tick | 16 + 16 | 2 x uint32 counters, 8 B; reader copies with interrupts off (DI/EI) | two 16-bit halves of one uint32, 4 B; one store, one load, wrap-safe differences | 8 -> 4 B (M, nm) | none (shared memory) | built, both forms (encoder.c: g_packed / g_count_l, g_count_r) |
| 2 | Missed-edge count, left and right | Buddy 2; no (no Week 6 form) | ENC_ISR -> motion_task | 0..65535, saturates | 16 + 16 | none | packed uint32, 4 B, one store | 4 -> 4 B | none | built (encoder.c: g_invalid_packed) |
| 3 | Per-phase edge counts LA, LB, RA, RB (FR-30) | Buddy 2; no (added at plan 0.3) | ENC_ISR -> motion_task | 0..65535, wraps | 4 x 16 | none | 2 packed uint32 (8 B) + 4 x uint16 ISR-private (8 B) | 16 -> 16 B | none | built (encoder.c: g_edges_la..rb, g_edges_l/r) |
| 4 | Decoder state per wheel (motion_quad_t) | Buddy 2; no | ENC_ISR only | state 0..3 | 2 | n/a | uint8 inside a 6 B struct; ISR working state, kept unpacked (reason) | 12 -> 12 B | none | built (motion_lib.h: motion_quad_t) |
| 5 | ISR timing counters | Buddy 2; no | ENC_ISR -> motion_task | test evidence | n/a | n/a | 3 x uint32; instrumentation, not application data | 12 -> 12 B | none | built (encoder.c: g_irq_count, g_irq_max_us, g_irq_last_us) |
| 5a | NFR-18 request stamps (g_stop_req_us, one per STOP source) | Buddy 2; no | STOP senders (scan_task, imu_task, the stand-in), core 1 -> motion_task | 1 MHz time, wraps | 32 each | n/a | 2 x uint32, one store each by its own sender; test evidence, not application data | 8 -> 8 B (C) | none | built (motor_task.c: g_stop_req_us, motion_stop_stamp) |
| 6 | distance_state.dist_mm | writer Buddy 2, reader Buddy 4; switch inside tick_distance() and motion_ds_read() | motion_task (core 1) -> imu_task | net mm, average of both wheels, modulo 32768; read by differences (motion_ds_delta_mm, correct while under 16384 mm between reads) | 15 | struct {int32 dist_mm; int16 turn_dps; uint8 state; uint8 counter}, 8 B, plus the mutex ID 4 B; priority-inheritance mutex, writer TMO_POL (skips the tick if busy), reader waits (held only to copy). State and counter fit in Week 6's padding, so the struct is still 8 B | one uint32 word (bits 0-14), 4 B, one store, one load, no mutex; reader is imu_task on core 1 only (PostTri1 row 2) | 12 -> 4 B (M, nm on both car builds of the working tree, 26 Oct: g_ds 8 + g_ds_mtxid 4 -> g_ds_word 4; measure.sh again after the commit), plus 1 of 4 kernel mutex slots in the before build only (fixed table, 0 B more) | none (shared memory) | built, both forms (motion_ds.h; motor_task.c: g_ds_word / g_ds, g_ds_mtxid) |
| 7 | distance_state.state | as row 6 | as row 6 | 0 STEADY, 1 ACCELERATING, 2 DECELERATING, 3 never sent; forward acceleration over 50 ms windows against 171 mm/s^2 (proposed) | 2 | uint8 in the struct (not in Week 6: Buddy 4's October request) | bits 15-16 | in row 6 | none | built |
| 8 | distance_state.update counter | as row 6 | as row 6 | 0..7, wraps; +1 per publish | 3 | uint8 in the struct (not in Week 6) | bits 17-19 | in row 6 | none | built |
| 9 | distance_state.turn_rate | as row 6 | as row 6 | deg/s, CCW positive, -2048..+2047, clamped; 50 ms window (proposed), 0.716 deg/s per count (C) | 12 | int16 in the struct | bits 20-31, offset +2048 (Barr-C 5.3: unsigned in the bit operations) | in row 6 | none | built |
| 10 | Command opcode (nav_cmd_mbf x4, avoid_cmd_mbf x2) | shared with Buddy 5; switch only if senders use motion's functions, else size | nav_ctrl_task / scan_task -> motion_task | 3-bit opcode; avoid uses reverse=1, bypass=2 | 3 | 16-bit packed command word | bits 0-2 | buffers 48 -> 48 B (6 x 8 B slots, C) | payload 4 -> 2 B; slot 8 -> 8 B | avoid buffer built; command consumer not built (1.7, 1.10) |
| 11 | Command side / fault source | as row 10 | as row 10 | avoid side: left=0, right=1; reverse side=0 | 1 | packed word | bit 3 | in row 10 | in row 10 | avoid command packer built |
| 12 | Command value | as row 10 | as row 10 | 1..4095 mm for avoid commands | 12 | packed word | bits 4-15 | in row 10 | in row 10 | avoid command packer built |
| 13 | Replies MOVE_DONE, CMD_REJECTED, MOTION_FAULT, LINE_FOUND, LINE_LOST (nav_flg); REVERSE_DONE, LINE_FOUND, LINE_LOST (scan_flg) | flags owned by Buddy 5; no | motion_task -> nav_ctrl_task / scan_task | set/clear | 1 each | 1 bit each in an event flag (already minimum in Week 6) | same | 0 B (fixed FLGCB table) | 0 B (not messages) | not built (bits defined in nav_ctrl.h) |
| 14 | motion_flg bits TICK, CMD, STOP_SCAN, STOP_IMU | Buddy 2; no | cyclic handler / nav / scan / imu -> motion_task | set/clear | 1 each | 1 bit each | same | 0 B | 0 B | built: TICK, STOP_SCAN, STOP_IMU used; CMD defined for step 1.7 (motor_task.h: MOTION_EVT_*) |
| 15 | Statechart 6.2 current state | Buddy 2; switch | motion_task only | 9 leaf states | 4 | C enum, 1 B on this compiler (M, short enums) | bits 0-3 of one uint8 | 2 -> 1 B (rows 15 + 16) | none | not built (1.3) |
| 16 | History state (H, leaving Stopped) | Buddy 2; switch | motion_task only | 9 states | 4 | enum, 1 B | bits 4-7 of the same uint8 | in row 15 | none | not built |
| 17 | Bench phase (motion_phase_t) | Buddy 2; no (test only) | motion_task | 5 values (stop test added) | 3 | n/a | enum, 1 B | 1 -> 1 B | none | built (motor_task.c: motion_phase_t) |
| 18 | Motor stall fault | flag Buddy 5, set by Buddy 2; no | motion_task -> nav_ctrl and others | 0/1 (OR of left and right) | 1 | MOTION_FAULT bit on nav_flg; cause kept private | fault_flg bit 0; brake first, then set | 0 B (fixed table) | 0 B | not built (1.11) |
| 19 | Encoder fault | as row 18 | as row 18 | 0/1 (OR of left and right) | 1 | as row 18 | fault_flg bit 1 | 0 B | 0 B | not built |
| 20 | Private fault byte (stall L/R, tick, init, encoder L/R) | Buddy 2; no | motion_task only | 6 flags | 6 | (today's code, not Week 6) | 6 bits in one uint8 | 1 -> 1 B | none | built (motor_task.h: MOTION_FAULT_*) |
| 21 | Telemetry motion event (whole record) | format Buddy 1; no; Buddy 1 measures | motion_task -> comms_task (core 0) via tele_mbf | 1 per second and on state change | 51 | v1 T_TELEMETRY_EVENT {UW t_ms; UINT kind; INT a, b} 16 B (20 B in mbf); wire: v1 JSON E line about 55 B (C) | v3 body: 7 B payload, 12 B in mbf, 13 B record E (C, from messages.h) | n/a (Buddy 1's buffer) | 16 / 20 / ~55 -> 7 / 12 / 13 B | sender not built (1.8); layout in messages.h (Buddy 1, COMM_W_MOTION_*) |
| 22 | Telemetry kind | Buddy 1 | as row 21 | 0 = motion | 3 | INT kind | 3 bits | in row 21 | in row 21 | n/a |
| 23 | Telemetry state (with turn direction) | Buddy 1 / 2 | as row 21 | 0..15 | 4 | a = 0..3 (INT) | 4 bits | in row 21 | in row 21 | not built |
| 24 | Telemetry speed_l, speed_r | Buddy 1 / 2 | as row 21 | -128..127 cm/s (PLACEHOLDER, E) | 8 + 8 | not in v1 | offset 8 bits each | in row 21 | in row 21 | not built |
| 25 | Telemetry dist_mm | Buddy 1 / 2 | as row 21 | 0..32767, wraps | 15 | b = cm since last event (INT) | 15 bits | in row 21 | in row 21 | not built |
| 26 | Telemetry turn_rate | Buddy 1 / 2 | as row 21 | -512..511 deg/s, offset | 10 | not in v1 | 10 bits | in row 21 | in row 21 | not built |
| 27 | Telemetry fault, ovf | Buddy 1 / 2 | as row 21 | 2 fault bits; ovf 0/1 (PLACEHOLDER) | 2 + 1 | not in v1 | 3 bits | in row 21 | in row 21 | not built |
| 28 | FR-23 encoder counts in telemetry | Buddy 1 / 2; open | as row 21 | TBD | TBD (proposal +32) | TBD | TBD | TBD | TBD | undecided |
| 29 | line_state as read by motion | Buddy 3; her own switch | ir_sense_task (1 ms) -> motion_task (10 ms) | few discrete values (digital sensors) | TBD by Buddy 3 | struct + priority-inheritance mutex | one word, one store; mutex TBD (PostTri1 row 2) | Buddy 3 measures | none | not built |
| 30 | hump_state as read by motion | Buddy 4; her own switch | imu_task -> motion_task, nav | NONE / CLIMBING / DESCENDING | 2 | enum (1 B) + priority-inheritance mutex | 2-bit field in one word, no mutex (22 Oct; row 2 to correct) | Buddy 4 measures | none | not built |
| 31 | PID working variables (motion_pid_t x2) | Buddy 2; no | motion_task only | int32 Q8 | n/a | n/a | 28 B each; working variables, kept unpacked (ImmTri1 "PID maths" row) | 56 -> 56 B | none | built (motion_lib.h: motion_pid_t) |
| 32 | Other motion_ctx_t working fields (positions, deltas, timing, the distance_state window) | Buddy 2; no | motion_task only | n/a | n/a | n/a | 236 B before distance_state, 304 B with it (M, nm, 26 Oct); the stop fields add 16 B more (C), re-measure | same in both builds | none | built (motor_task.c: motion_ctx_t g_ctx) |
| 33 | Motor driver table g_motor | Buddy 2; no | motor.c | pin and slice configuration | n/a | n/a | 32 B .data | 32 -> 32 B | none | built (motor.c: g_motor) |
| 34 | Tunables (motion_cfg.h) | Buddy 2; no | compile time | n/a | n/a | n/a | #define constants | 0 -> 0 B | none | built |
| 35 | Motion print record (console channel, every build) | Buddy 2; no (the same in both; carries the timing evidence) | motion_task -> motion_report (core 0) | n/a | n/a | n/a | 52 B record; 8 slots x 56 B = 448 B | 448 -> 448 B | 52 / 56 -> same | built; planned to shrink after PID tuning (motor_task.c: motion_report_t, g_report_buf) |
| 36 | Task stacks motion_task, motion_report | Buddy 2; no | n/a | n/a | n/a | n/a | 2 x 4096 B static (built (motor_task.c: g_motion_stack, g_report_stack)) | 8192 -> 8192 B | none | built (motor_task.c: g_motion_stack, g_report_stack) |
| 37 | Kernel objects | Buddy 2; no | n/a | n/a | n/a | n/a | 2 TCB, 1 FLGCB, 1 MBFCB, 1 CYCCB; later 2 MBF for commands, plus 1 MTX in the before build only; fixed tables (config/config.h CNF_MAX_*) | 0 B change | none | partly built |

**Kept unpacked on purpose.** Working variables used only inside one task
(PID state, positions, timing, the decoder state) stay in natural C types:
packing them would add shifts and overflow risk with no shared-data or
message saving (Tribunal 1, ImmTri1 "PID maths" row). Fallback rule
(ImmTri1): if a field cannot be packed safely, it is unpacked and the
reason recorded here.

**Where the packed form differs from what Tribunal 1 wrote** (record in the
Week 13 write-up):
- The encoder interrupt publishes both counts with one 32-bit store;
  ImmTri1 said each 16-bit half with its own store. One store is at least
  as safe: a reader can never see half an update.
- The encoder interrupt builds the packed words itself; ImmTri1 R5 said
  interrupts only count and tasks pack. It costs a few shifts per edge and
  keeps every published word a single store.
