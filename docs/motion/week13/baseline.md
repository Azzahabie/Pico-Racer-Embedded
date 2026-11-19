# Week 13 evidence 2 and 3: motion's RAM and message sizes

The changed requirement (Tribunal 1) asks for RAM and message sizes before
and after the minimum-bit change, and evidence that timing did not get
unacceptably worse. This file is motion's (Buddy 2's) share. Every number
says how it was obtained: M = measured, C = calculated, E = estimated.

## Final implementation (what Week 13 needs since 27 Oct)

The prof's 27 Oct announcement: comparing with the original design is no
longer required; give evidence for the final implementation only. The
final numbers, measured with `tests/week13/measure.sh` on commit a7ee725
(car profile, `MOTION_PACKED` 1, 0 warnings;
[measure_a7ee725.txt](measure_a7ee725.txt)):

| What | Final | How |
|---|---|---|
| Motion static RAM | 9369 B (36 B .data + 9333 B .bss), of which 8192 B are the two task stacks and 568 B the motion_task context g_ctx | M, `arm-none-eabi-size` and `nm` |
| Whole image static RAM | 240 B .data + 32108 B .bss | M, `arm-none-eabi-size -A` |
| Command buffers | nav_cmd_mbf 32 B (room 4), avoid_cmd_mbf 16 B (room 2) | M, nm |
| E motion record | 7 B payload, 12 B in tele_mbf, 13 B record E | C, messages.h (`COMM_W_INTAKE_SIZE_MOTION`, `COMM_W_E_SIZE_MOTION`) and the 4 B header + round-to-4 rule; cross-checked by the car's ARM compiler in `tests/week13/sizes.c` |
| Avoid command | 2 B payload (packed uint16) | C, messages.h |
| Nav command | 2 B payload (2 raw bytes, INTEGRATION.md D4) | C, nav_ctrl.h |

Per-field detail: [representation.md](representation.md). Re-run
measure.sh on the demo commit. Timing evidence (correctness and timing,
item 4) comes from the 5 s timing line on the car, see
`docs/motion/README.md` "Timing evidence".

The rest of this file is the before/after method and numbers from October,
kept as history.

## Before and after, on the same code

`MOTION_PACKED` in `app_program/motor_task/motion_cfg.h` selects the
layout:

- **1, after** (the only value ever committed): the packed layouts.
- **0, before**: the Week 6 layouts, rebuilt from the same commit only to
  measure them. The encoder word was packed from its first commit, so this
  "before" never shipped; it is the Week 6 design built for comparison.

Building both from one commit means RAM, message sizes and timing compare
like for like (PostTri1 Version C row 8 promises the stop-time test on both
builds).

In the final code the switch covers only the encoder counts and
`distance_state`. The commands, the statechart byte and the E motion
record were built without a Week 6 form and are the same in both builds.

## Method

```sh
git commit ...                     # it measures the last commit only
sh tests/week13/measure.sh > out.txt
sh tests/week13/measure.sh -DMOTION_BENCH=1   # bench images for car timing
```

The script copies the last commit into two throwaway folders, builds the
car profile in the Docker image with `-DMOTION_PACKED=1` and `=0` on the
compiler line (motion_cfg.h is never edited for this), and prints:

- the whole image's `.data` and `.bss` (`arm-none-eabi-size -A`);
- the size of each motion object in `app_program/motor_task/`;
- every RAM variable in those objects (`nm -S`), and which ones differ;
- message sizes worked out by the car's ARM compiler (`tests/week13/sizes.c`).
  Today's entries are plain constant expressions, so they are calculated
  (C); `sizes.c` matters once it holds structs or enums, where a PC's
  `sizeof` can differ (this ABI stores an enum in one byte).

**Counting rule.** Motion's RAM is the `.data` plus `.bss` of its own
objects. Both task stacks and the report queue are static arrays
(`TA_USERBUF`), so they are inside that number; nothing of motion's comes
from the kernel heap. Kernel control blocks sit in fixed tables sized in
`config/config.h`, so creating or dropping a flag, mutex or buffer changes
no byte in the map: motion uses 2 task, 1 flag, 3 message buffer
(report queue, nav_cmd_mbf, avoid_cmd_mbf) and 1 cyclic handler slots,
plus 1 mutex in the `MOTION_PACKED` 0 build only. Buddy 5's `gpio_irq.o` is not counted. The whole
image is shown as well, but it can move by linker alignment.

**Message sizes.** A message buffer stores each message as a 4-byte
header plus the message rounded up to 4 bytes
(`kernel/tkernel/messagebuf.h`), so the RAM a message costs is not its
`sizeof`. Both are reported.

## Results, 24 Oct (M, commit 292860e, car profile)

Raw output: [measure_292860e.txt](measure_292860e.txt).

| | Packed (1, after) | Unpacked (0, before) | Change |
|---|---|---|---|
| Encoder counts | `g_packed` 4 B | `g_count_l` + `g_count_r` 8 B | 8 -> 4 B |
| Motion RAM (data + bss of its objects) | 8969 B | 8973 B | -4 B |
| Motion code (text), all objects | 7205 B | 7197 B | +8 B |
| of which encoder.o (the packing) | 616 B | 612 B | +4 B |
| of which motor_task.o (the ready line's `MOTION_PACKED` stamp, not packing) | 5313 B | 5309 B | +4 B |
| Whole image `.bss` | 29104 B | 29112 B | -8 B (alignment) |
| Builds, car and default profile | 0 warnings | 0 warnings | |

So packing the counts saves 4 B of RAM and costs 4 B of code (the
interrupt calls `motion_pack_counts`), and the reader needs no interrupt
masking. The other 4 B of code is the ready line printing which build it
is. The motion RAM total is dominated by the two 4096 B task stacks and
motion's 448 B print queue (motion_task to motion_report on core 0, in
every build), which are the same in both builds.

Before the stacks were made static (commit 3998a8d), motion's static RAM
was 777 B, and the kernel took another 8464 B from its heap for the two
stacks (C: 2 x (4096 B stack + 128 B system stack + 8 B heap block
header), kernel/tkernel/task_manage.c and memory.c); the map could not
show those.

**Messages (October; the E motion record has existed since 754fdf7, 31 Oct; on master 1 Nov via PR #18).**

| Message | Before | After |
|---|---|---|
| Motion telemetry event (Buddy 1's schema v3, not switched by motion) | Buddy 1 measures v1 | 7 B payload, 12 B in the message buffer, 13 B on the wire (C: `COMM_W_BYTES(51)` = 7; 4 B header + 7 rounded to 8 = 12; `COMM_W_BYTES(53 + 51)` = 13, all in messages.h) |
| Motion print record (motion_task to motion_report, every build) | 52 B, 448 B queue | the same |

The print record is motion's console channel, present in every build
including the integration image, and the same in both settings, so it is
not a "before" for anything. With its 4096 B stack it costs 4544 B.

## October plan (all built since; final forms in representation.md)

| Item (plan step) | Switched? | Expected before -> after | Owner |
|---|---|---|---|
| Commands on nav_cmd_mbf and avoid_cmd_mbf (1.7) | if Buddy 5 sends through motion's functions; otherwise size only | payload 4 -> 2 B; buffer slot 8 -> 8 B, 6 slots 48 -> 48 B (C) | Buddy 2 with Buddy 5 |
| distance_state (1.4), built 26 Oct | yes, inside tick_distance() and motion_ds_read() | struct 8 B + mutex ID 4 B -> one word 4 B (M, nm on the working tree; re-run measure.sh after the commit) | Buddy 2, read by Buddy 4 |
| Motion state and history (1.3) | yes | 2 enums 2 B -> one byte (C) | Buddy 2 |
| Telemetry sender (1.8) | no (Buddy 1's format) | Buddy 1's v1 record -> 7 B event | Buddy 1, motion supplies fields |
| Shared fault bits (1.11, 2.4) | no | event-flag bits in both | Buddy 5's flag, motion sets its bits |

In the end only `distance_state` got a Week 6 form; the commands, the
statechart byte, the telemetry sender and the fault bits were built
packed only (see [representation.md](representation.md)).

## Needs the car (October plan; since 27 Oct the both-builds runs are optional)

- **NFR-11 timing on both builds:** run the bench test on
  `motion_packed0.uf2` and `motion_packed1.uf2` from the script (same
  commit, same duty, wheels off, 20 timing windows each) and keep the ready
  line, which ends with `MOTION_PACKED=0` or `=1`, in each log.
- **NFR-18 stop time on both builds,** once the STOP bits exist.
- **With Wi-Fi on,** which needs the Wi-Fi profile with motion integrated.
- **Expected difference (E, from the disassembly, not measured):**
  - Reader: about 35 CPU cycles per read packed against 38 unpacked, at
    100 reads a second, so about 300 cycles a second in packing's favour.
  - Interrupt: publishing is about 13 cycles packed (it calls
    `motion_pack_counts`) against 8 unpacked (two stores), about 5 cycles
    more per edge. At about 6800 edges a second (both wheels at
    350 per-mille, results.md) that is about 34,000 cycles a second, so
    overall the packed build is expected to use slightly more CPU (about
    0.03 % of a 125 MHz core), not less.
  - The unpacked read keeps interrupts off for about 11 cycles, which can
    only delay an encoder interrupt, not cost CPU. The timing line's
    encoder "longest" cannot show that, because it starts at the
    dispatcher's entry; compare the tick work maximum and the stop test.
