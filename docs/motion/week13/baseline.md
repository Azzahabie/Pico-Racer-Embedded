# Week 13 evidence 2 and 3: motion's RAM and message sizes, before and after

The changed requirement (Tribunal 1) asks for RAM and message sizes before
and after the minimum-bit change, and evidence that timing did not get
unacceptably worse. This file is motion's (Buddy 2's) share. Every number
says how it was obtained: M = measured, C = calculated, E = estimated.

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

Today the switch covers the encoder counts. Each later feature (commands,
`distance_state`, the motion state) is written with both forms under the
same switch; see "To be added".

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
- message sizes compiled by the car's ARM compiler (`tests/week13/sizes.c`;
  a PC's `sizeof` can differ, since this ABI stores an enum in one byte).

**Counting rule.** Motion's RAM is the `.data` plus `.bss` of its own
objects. Both task stacks and the report queue are static arrays
(`TA_USERBUF`), so they are inside that number; nothing of motion's comes
from the kernel heap. Kernel control blocks sit in fixed tables sized in
`config/config.h`, so creating or dropping a flag, mutex or buffer changes
no byte in the map: motion uses 2 task, 1 flag, 1 message buffer and
1 cyclic handler slots. Buddy 5's `gpio_irq.o` is not counted. The whole
image is shown as well, but it can move by linker alignment.

**Message sizes.** A message buffer stores each message as a 4-byte
header plus the message rounded up to 4 bytes
(`kernel/tkernel/messagebuf.h`), so the RAM a message costs is not its
`sizeof`. Both are reported.

## Results now (M, commit 292860e, car profile)

Raw output: [measure_292860e.txt](measure_292860e.txt).

| | Packed (1, after) | Unpacked (0, before) | Change |
|---|---|---|---|
| Encoder counts | `g_packed` 4 B | `g_count_l` + `g_count_r` 8 B | 8 -> 4 B |
| Motion RAM (data + bss of its objects) | 8969 B | 8973 B | -4 B |
| Motion code (text) | 7205 B | 7197 B | +8 B |
| Whole image `.bss` | 29104 B | 29112 B | -8 B (alignment) |
| Builds, car and default profile | 0 warnings | 0 warnings | |

So packing the counts saves 4 B of RAM and costs 8 B of code, plus a
read that needs no interrupt masking. The motion RAM total is dominated by
the two 4096 B task stacks and the 448 B bench report queue, which are the
same in both builds.

Before the stacks were made static (commit 3998a8d), motion's static RAM
was 777 B, and the kernel took another 8464 B from its heap for the two
stacks (C: 2 x (4096 B stack + 128 B system stack + 8 B heap block
header), kernel/tkernel/task_manage.c and memory.c); the map could not
show those.

**Messages today.** None of motion's Week 13 messages exist yet.

| Message | Before | After |
|---|---|---|
| Motion telemetry event (Buddy 1's schema v3, not switched by motion) | Buddy 1 measures v1 | 7 B payload, 12 B in the message buffer, 13 B on the wire (M, sizes.c) |
| Bench report record (test channel only) | 52 B, 448 B queue | the same |

The bench report record is a test channel, the same in both builds; it is
not the Week 13 "before".

## To be added as features land

| Item (plan step) | Switched? | Expected before -> after | Owner |
|---|---|---|---|
| Commands on nav_cmd_mbf and avoid_cmd_mbf (1.7) | if Buddy 5 sends through motion's functions; otherwise size only | payload 4 -> 2 B; buffer slot 8 -> 8 B, 6 slots 48 -> 48 B (C) | Buddy 2 with Buddy 5 |
| distance_state (1.4) | yes, through motion's read and publish functions | struct + mutex 8 B -> one word 4 B (C) | Buddy 2, read by Buddy 4 |
| Motion state and history (1.3) | yes | 2 enums 2 B -> one byte (C) | Buddy 2 |
| Telemetry sender (1.8) | no (Buddy 1's format) | Buddy 1's v1 record -> 7 B event | Buddy 1, motion supplies fields |
| Shared fault bits (1.11, 2.4) | no | event-flag bits in both | Buddy 5's flag, motion sets its bits |

Each one adds its Week 6 form under `MOTION_PACKED`, a line in `sizes.c`
where it is a message, and a row update in
[representation.md](representation.md).

## Needs the car

- **NFR-11 timing on both builds:** run the bench test on
  `motion_packed0.uf2` and `motion_packed1.uf2` from the script (same
  commit, same duty, wheels off, 20 timing windows each) and keep the ready
  line, which ends with `MOTION_PACKED=0` or `=1`, in each log.
- **NFR-18 stop time on both builds,** once the STOP bits exist.
- **With Wi-Fi on,** which needs the Wi-Fi profile with motion integrated.
- **Expected difference (E):** about 35 against 38 CPU cycles per encoder
  read, and about 11 cycles with interrupts off in the unpacked build (from
  the disassembly, not measured). The timing line's encoder "longest" cannot
  show that interrupts-off window, because it starts at the dispatcher's
  entry; compare the tick work maximum and the stop test instead.
