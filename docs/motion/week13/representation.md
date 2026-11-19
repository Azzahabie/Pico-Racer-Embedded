# Week 13 evidence 1: data representation table (motion, Buddy 2)

One row per piece of application data motion owns, writes or reads: its
valid range, the bits it needs, and how the final implementation stores and
shares it. Since the prof's 27 Oct announcement, Week 13 needs evidence for
the final implementation only, so the RAM and message columns give the
final build; the "Week 6 form" column is kept only to show what the
minimum-bit change replaced.

Legend: M = measured, E = estimated, C = calculated. "nm" = measured with
`arm-none-eabi-nm -S` by `tests/week13/measure.sh` on commit e47c4d8, car
profile (SMP=1 CONSOLE=usb_cdc), `MOTION_PACKED` 1, 0 warnings; its output
is in [measure_e47c4d8.txt](measure_e47c4d8.txt). Re-run it on the demo
commit.

| # | Field | Owner | Writer -> reader(s), core | Valid range (M/E) | Bits | Week 6 form | Final form, size, sync | RAM (final) | Message bytes (final) | Status |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Encoder count, left and right | Buddy 2 | ENC_ISR -> motion_task, both core 1 | 0..65535, wraps; 34 per tick at 350 per-mille (M, 24 Oct); wraps only above 32767 per tick | 16 + 16 | 2 x uint32 counters, 8 B; reader copies with interrupts off | two 16-bit halves of one uint32; one store, one load, wrap-safe differences | 4 B (M, nm: g_packed) | none (shared memory) | built (encoder.c) |
| 2 | Missed-edge count, left and right | Buddy 2 | ENC_ISR -> motion_task | 0..65535, saturates | 16 + 16 | none | packed uint32, one store | 4 B (M, nm) | none | built (encoder.c: g_invalid_packed) |
| 3 | Per-phase edge counts LA, LB, RA, RB (FR-30) | Buddy 2 | ENC_ISR -> motion_task | 0..65535, wraps | 4 x 16 | none | 2 packed uint32 + 4 x uint16 ISR-private | 16 B (M, nm) | none | built (encoder.c: g_edges_*) |
| 4 | Decoder state per wheel (motion_quad_t) | Buddy 2 | ENC_ISR only | state 0..3 | 2 | n/a | uint8 inside a 6 B struct; ISR working state, kept unpacked (see below) | 12 B (M, nm) | none | built (motion_lib.h) |
| 5 | ISR timing counters | Buddy 2 | ENC_ISR -> motion_task | test evidence | n/a | n/a | 3 x uint32; instrumentation, not application data | 12 B (M, nm) | none | built (encoder.c) |
| 5a | NFR-18 request stamps, one per STOP source | Buddy 2 | STOP senders (scan_task, imu_task) -> motion_task | 1 MHz time, wraps | 32 each | n/a | 2 x uint32, one store each by its own sender; test evidence | 8 B (M, nm: g_stop_req_us) | none | built (motor_task.c: motion_stop_stamp) |
| 6 | distance_state.dist_mm | writer Buddy 2, readers Buddy 4 and 5 | motion_task (core 1) -> imu_task and scan_task (both core 1; scan_task for the pivot-aware re-plan, INTEGRATION.md D13) | net mm, average of both wheels, modulo 32768; read by differences (motion_ds_delta_mm, correct while under 16384 mm between reads) | 15 | struct {int32 dist_mm; int16 turn; uint8 state; uint8 counter} 8 B, plus a priority-inheritance mutex | one uint32 word (bits 0-14), one store, one load, no mutex | 4 B for rows 6-9 (M, nm: g_ds_word) | none (shared memory) | built (motion_ds.h; motor_task.c: tick_distance) |
| 7 | distance_state.state | as row 6 | as row 6 | 0 STEADY, 1 ACCELERATING, 2 DECELERATING, 3 never sent; forward acceleration over 50 ms windows against 171 mm/s^2 (proposed) | 2 | uint8 in the struct | bits 15-16 | in row 6 | none | built |
| 8 | distance_state.update counter | as row 6 | as row 6 | 0..7, wraps; +1 per publish | 3 | uint8 in the struct | bits 17-19 | in row 6 | none | built |
| 9 | distance_state.turn_rate | as row 6 | as row 6 | deg/s, counter-clockwise positive, -2048..+2047, clamped; 50 ms window, 0.716 deg/s per count (C) | 12 | int16 in the struct | bits 20-31, offset +2048 (Barr-C 5.3: unsigned in the bit operations) | in row 6 | none | built |
| 10 | Avoid command (avoid_cmd_mbf): op, side, distance | format Buddy 2 and 5 (messages.h) | scan_task (core 1) -> motion_task | op 1 REVERSE, 2 BYPASS, 3 LOOK; side left/right (BYPASS only); 1..511 mm | 3 + 1 + 9 (3 reserved) | 4-byte struct, mm | one uint16 built with shifts and masks (motion_avoid_cmd_pack/unpack, which reject anything outside the ranges) | buffer 16 B, room 2 (M, nm: g_avoid_buf) | payload 2 B | built (5ac5c1b; used on the car in S7) |
| 10a | Nav command (nav_cmd_mbf): op, argument | format Buddy 5 (nav_ctrl.h) | nav_ctrl_task (core 0) -> motion_task | op JUNCTION (arg 0 FORWARD, 1 LEFT, 2 RIGHT, 3 U_TURN) or FAULT_CLEAR (arg 0x01 IMU, 0x02 MOTION) | needs 1 + 2; uses 8 + 8 | 4-byte struct, mm | 2 raw bytes { NAV_CMD_*, arg }, not bit-packed (INTEGRATION.md D4, open) | buffer 32 B, room 4 (M, nm: g_nav_buf) | payload 2 B | built (5ac5c1b); JUNCTION never run on the car |
| 11 | (merged into rows 10 and 10a) | | | | | | | | | |
| 12 | (merged into rows 10 and 10a) | | | | | | | | | |
| 13 | Replies MOVE_DONE, CMD_REJECTED, MOTION_FAULT, LINE_FOUND, LINE_LOST (nav_flg); REVERSE_DONE, LINE_FOUND, LINE_LOST (scan_flg) | flags owned by Buddy 5 | motion_task -> nav_ctrl_task / scan_task | set/clear | 1 each | 1 bit each in an event flag (already minimum in Week 6) | same | 0 B (fixed kernel table) | 0 B (not messages) | built (motor_task.c: sm_send_replies) |
| 14 | motion_flg bits TICK, CMD, STOP_SCAN, STOP_IMU | Buddy 2 | cyclic handler / senders / scan / imu -> motion_task | set/clear | 1 each | 1 bit each | same (common/messages.h: MOTION_EVT_*) | 0 B | 0 B | built, all four used |
| 15 | Statechart 6.2 current state | Buddy 2 | motion_task only | 9 leaf states | 4 | n/a | bits 0-3 of one uint8 (motion_sm_t.packed) | in g_ctx (row 32) | none | built (motion_sm.h) |
| 16 | History state (H, leaving Stopped) | Buddy 2 | motion_task only | the 5 Active states or none (0xF) | 3 (stored in 4, one nibble) | n/a | bits 4-7 of the same uint8 | in g_ctx | none | built |
| 16a | Statechart flags: turn owed a reply, bypass waiting, search after a bypass | Buddy 2 | motion_task only | 3 flags | 3 | n/a | bits 0-2 of a second uint8 (motion_sm_t.flags) | in g_ctx | none | built (MOTION_SM_F_*) |
| 17 | Bench phase (motion_phase_t) | Buddy 2 (test only) | motion_task | 5 values | 3 | n/a | enum, 1 B | in g_ctx | none | built |
| 18 | Motor stall fault | flag Buddy 5, set by Buddy 2 | motion_task -> nav_ctrl and comms (S.faults) | 0/1 (OR of left and right) | 1 | MOTION_FAULT bit on nav_flg; cause kept private | fault_flg bit FAULT_MOTOR_STALL; brake first, then set | 0 B (fixed table) | in S.faults | built (5ac5c1b; motor_task.c: fault_flg_bits) |
| 19 | Encoder fault | as row 18 | as row 18 | 0/1 (OR of left and right) | 1 | as row 18 | fault_flg bit FAULT_ENCODER | 0 B | in S.faults | built |
| 20 | Private fault byte (stall L/R, tick, init, encoder L/R) | Buddy 2 | motion_task only | 6 flags | 6 | n/a | 6 bits in one uint8 (motor_task.h: MOTION_FAULT_*) | in g_ctx | none | built |
| 21 | Telemetry: E motion record (whole) | format Buddy 1 (MQTT_SCHEMA.md 5.1), sender Buddy 2 | motion_task -> comms_task (core 0) via tele_mbf | one on every statechart change and one a second while a wheel moves (without it comms_task's S.dst, cm since the last motion change, stays 0); send never waits, refused sends counted on the 5 s timing line and tried again 100 ms later | 51 | v1 T_TELEMETRY_EVENT 16 B (20 B in mbf); JSON E line about 55 B (C) | packed body: comm_w_pack_motion | n/a (Buddy 1's buffer) | 7 B payload, 12 B in tele_mbf, 13 B record E (C, messages.h; cross-checked by the ARM compiler in tests/week13/sizes.c) | built (754fdf7; extended 19 Nov: window speeds, drop count, once a second while moving, mission distance) |
| 22 | Telemetry kind | Buddy 1 | as row 21 | 0 = motion | 3 | INT kind | 3 bits | in row 21 | in row 21 | built |
| 23 | Telemetry state | Buddy 1 / 2 | as row 21 | 0..8 used (motion_state_t), 0..15 room | 4 | a = 0..3 (INT) | 4 bits | in row 21 | in row 21 | built |
| 24 | Telemetry speed_l, speed_r | Buddy 1 / 2 | as row 21 | -128..127 cm/s, stored +128 (confirmed by Buddy 2); far inside it: about 27 cm/s at 35 % power with no load (M/C, 24 Oct), about 4.7 cm/s line following (C, INTEGRATION.md S4); each wheel over the 50 ms distance_state window, rounded | 8 + 8 | not in v1 | offset 8 bits each | in row 21 | in row 21 | built |
| 25 | Telemetry dist_mm | Buddy 1 / 2 | as row 21 | 0..32767, wraps: mm travelled in this mission, forward and back alike, so it never goes down (0 at a new mission, kept on a resume; Section 4); from both wheels' counts | 15 | b = cm since last event (INT) | 15 bits | in row 21 | in row 21 | built |
| 26 | Telemetry turn_rate | Buddy 1 / 2 | as row 21 | -512..511 deg/s, stored +512 | 10 | not in v1 | 10 bits | in row 21 | in row 21 | built |
| 27 | Telemetry fault, ovf | Buddy 1 / 2 | as row 21 | 2 fault bits (stall, encoder); ovf 0/1 = a field was clamped | 2 + 1 | not in v1 | 3 bits | in row 21 | in row 21 | built |
| 28 | FR-23 encoder counts in telemetry | Buddy 1 / 2 | as row 21 | n/a | proposal: 0 | n/a | proposal: not sent separately; dist_mm, speed_l and speed_r are computed from them (needs Buddy 2's approval as a deviation) | n/a | n/a | open (MQTT_SCHEMA.md item 16) |
| 29 | line_state as read by motion | Buddy 3 | ir_sense_task (1 ms, core 1) -> motion_task (10 ms) | rl 0..3, steer 0..2, barcode/lost/fault/cal flags, counter 0..255, error -100..+100 (+128), dark 0..255 x 10 ms | 32 | struct + priority-inheritance mutex | one uint32 word, one store, one load per tick, no mutex (line_task/line_state.h) | Buddy 3's; motion keeps a copy in g_ctx | none | built |
| 30 | hump_state as read by motion | Buddy 4 | imu_task -> motion_task | NONE / CLIMBING / DESCENDING | 2 | enum (1 B) + priority-inheritance mutex | 2-bit field in one word, no mutex (imu_task.h, INTEGRATION.md D1) | Buddy 4's | none | built; the hump speed cap has not fired on the car yet |
| 31 | PID working variables (motion_pid_t x2) | Buddy 2 | motion_task only | int32 Q8 | n/a | n/a | 28 B each; working variables, kept unpacked | in g_ctx | none | built (motion_lib.h) |
| 32 | motion_ctx_t g_ctx: every working field (positions, speeds, timing, distance_state window, statechart, line following, turn, reverse, bypass) | Buddy 2 (with integration fields from Buddies 1, 3, 5) | motion_task only | n/a | n/a | n/a | one static struct | 560 B (M, nm) | none | built |
| 33 | Motor driver table g_motor | Buddy 2 | motor.c | pin and slice configuration | n/a | n/a | 32 B .data | 32 B (M, nm) | none | built |
| 34 | Tunables (motion_cfg.h) | Buddy 2 | compile time | n/a | n/a | n/a | #define constants | 0 B | none | built |
| 35 | Motion print record (console channel) | Buddy 2 | motion_task -> motion_report (core 0) | n/a | n/a | n/a | 52 B record; 8 slots x 56 B; carries the timing evidence | 448 B (M, nm: g_report_buf) | 52 / 56 B (console only) | built |
| 36 | Task stacks motion_task, motion_report | Buddy 2 | n/a | n/a | n/a | n/a | 2 x 4096 B static | 8192 B (M, nm) | none | built |
| 37 | Kernel objects | Buddy 2 | n/a | n/a | n/a | n/a | 2 TCB, 1 FLGCB, 3 MBFCB (report, nav_cmd, avoid_cmd), 1 CYCCB; fixed tables (config/config.h CNF_MAX_*) | 0 B beyond the tables; IDs 4 B each in .bss | none | built |

**Motion's static RAM, final build (M, nm, e47c4d8):** 36 B .data + 9325 B
.bss = 9361 B, of which 8192 B are the two task stacks and 560 B the
motion_task context. Whole image: 240 B .data + 32100 B .bss.

**Kept unpacked on purpose.** Working variables used only inside one task
(PID state, positions, timing, the decoder state) stay in natural C types:
packing them would add shifts and overflow risk with no shared-data or
message saving (Tribunal 1, ImmTri1 "PID maths" row). Fallback rule
(ImmTri1): if a field cannot be packed safely, it is unpacked and the
reason recorded here.

**Where the final form differs from what Tribunal 1 wrote** (for the
Week 13 write-up):
- The encoder interrupt publishes both counts with one 32-bit store;
  ImmTri1 said each 16-bit half with its own store. One store is at least
  as safe: a reader can never see half an update.
- The encoder interrupt builds the packed words itself; ImmTri1 R5 said
  interrupts only count and tasks pack. It costs a few shifts per edge and
  keeps every published word a single store.
- Nav commands are 2 raw bytes, not a packed 16-bit word (row 10a,
  INTEGRATION.md D4). Same 2 B payload; the bit-packing is not done.
