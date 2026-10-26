# Motion bench results (Buddy 2)

Dated log of measurements on the car. Every number here is measured on the
car unless it says otherwise; the method is given with it. Procedure:
[README.md](README.md).

## 2026-10-23 to 24: first bench session, wheels off the ground

**Set-up.** Branch `buddy2/motion` at 4e90726, built as a local bench image
(`MOTION_BENCH` 1, never committed) in the USB-CDC car profile. Car on a box
with the wheels off the ground, powered by the power bank, USB to the
laptop through a hub, serial output logged on the laptop.

### Wiring found and fixed

- **Encoder cables were swapped.** Turning the car's left wheel moved the
  right count. The two cables were swapped back so the car matches the
  approved pin map: left encoder on Grove 1 (GP0/GP1), right on Grove 2
  (GP2/GP3). The motors were already on the right drivers (left M1, right
  M2).
- **Left encoder cable came unplugged** while the cables were tidied. The
  left count then stayed at 0 and every `L fwd` step ended in
  `FAULT 0x01: left wheel stalled` with the motors braked. That is FR-30
  working on the car for a fully dead encoder: no edges, so it is reported
  as a motor stall (the stated limit). Replugging the cable fixed it.

### Encoders (assumption A2)

| Item | Left | Right | Method |
|---|---|---|---|
| Counts per wheel revolution, trial 1 | 2535 | 2533 | 10 slow hand turns against a tyre mark, change in count / 10 |
| Counts per wheel revolution, trial 2 | 2535.6 | 2536.8 | the same, after the flags were set (25,356 and 25,368 counts) |
| Count when the wheel turns forward, before flags | up | down | hand turn |
| Missed transitions while turning by hand | 0 | 13 in 25,334 | `missed` in the status line |

A third trial (A2 asks for 3) was lost when the USB link dropped; it is
still to do. The two trials agree to within 0.15 %.

The right encoder's few missed transitions followed it when its cable
moved from Grove 1 to Grove 2, so they come from that encoder or its cable,
not the Pico input. It is too small to matter for distance (about 0.05 %),
but the cable should be checked.

### Polarity (bench test step 2)

| Step | Before | After setting the flags |
|---|---|---|
| `L fwd` | left wheel forward, count +5117 | left wheel forward, count +5129 |
| `R fwd` | right wheel **backward**, count +5009 | right wheel forward, count +5026 |

So `MOTION_MOTOR_R_INVERT` and `MOTION_ENC_R_INVERT` are 1 (the right motor
and its encoder are both mounted mirrored); the left flags stay 0. Check
run: every `fwd` step turns its wheel forward with the count going up.

### Stall check (FR-30), wheel held by hand

| Wheel held | Result |
|---|---|
| Left, during `L fwd` (two runs) | `FAULT 0x01: left wheel stalled`, motors braked; the wheel crept 63 and 36 counts before the grip stopped it |
| Right, during `R fwd` (left steps ran normally first) | `FAULT 0x02: right wheel stalled`, motors braked; it crept 95 counts first |

Both were reported as a motor stall, not an encoder fault, so a held wheel
is not mistaken for a dead encoder wire. The fault comes 300 ms
(`MOTION_STALL_TICKS`, proposed) after the wheel stops making progress; the
log cannot time it more finely than that. Still to do: the one-wire
unplugged test (README, test 2).

### Open loop, dead band and timing (no load, wheels off the ground)

| Item | Left | Right |
|---|---|---|
| Speed at 350 per-mille | about 3420 counts/s (34 per tick) | about 3350 counts/s (33 per tick) |
| Dead band, three runs (per-mille) | 100, 125, 125 | 50, 25, 100 |

`MOTION_DEADBAND_DUTY` stays 150, which covers every no-load run; it must be
measured again on the floor, where the load is higher.

Core 1 timing during the motor runs (5 s windows): tick period
9.78 to 10.24 ms, tick work at most 232 us, encoder interrupts up to about
2960 per second, longest encoder callback 27 us (measured from the
dispatcher's entry stamp, a lower bound), no lost ticks.

### Closed-loop speed (bench test step 4, no load)

Bench image with `MOTION_TEST_PID` 1 and `MOTION_PID_TARGET` 16 counts per
tick (1600 counts/s, about half the 350 per-mille speed), gains as
committed (Kp 2048 and Ki 256 in Q8, Kd 0, output cap 600 per-mille).
11 runs of 3 s, all ended `DONE` with no fault.

| Item | Left | Right |
|---|---|---|
| Mean speed after the first second (109 samples, 200 ms each) | 1598.8 counts/s | 1600.3 counts/s |
| Standard deviation of those samples | 9.3 counts/s | 3.8 counts/s |
| Time to reach 97.5 % of target | 0.6 to 0.8 s | 0.6 to 0.8 s |
| Highest sample in any run | 1640 (2.5 % over, one run) | 1615 |
| Duty needed once settled | 158 to 180 per-mille | 154 to 184 per-mille |

No surging or oscillation was seen. These gains are a starting point for
the floor, where the load is higher; the PID tuning report needs logged
step responses on the floor too.

## 2026-10-24: wheel size, normal build check

**Ruler measurements (M, by hand, about +-1 mm):** wheel diameter 64 mm,
wheelbase (centre of left tyre to centre of right tyre) 127 mm.

**Calculated from them and the counts per revolution (C):**

| Item | Value |
|---|---|
| Wheel circumference | 201.1 mm |
| Counts per mm | 12.61 (0.079 mm per count) |
| 3400 counts/s (35 % power, no load) | 27.0 cm/s |
| 1600 counts/s (closed-loop test target) | 12.7 cm/s |
| 1 cm/s in counts | 126 counts/s |
| Pivot turn, each wheel in opposite directions | 90 deg = 1258 counts, 180 deg = 2515, 360 deg = 5030 |
| Turn rate from a 1-count left/right difference | 3.6 deg/s over 10 ms, 0.72 deg/s over 50 ms |
| 16-bit encoder count wraps after | 5.2 m (only differences are used) |

A 1 mm error in the diameter is a 1.6 % distance error, so the NFR-04
straight trials (500 mm on the floor) should calibrate counts per mm
before the turn and distance numbers are relied on.

**Normal build on the car (commit 4a69a12, MOTION_BENCH 0):** flashed and
run; motion_task and motion_report run on their static stacks
(3998a8d), ticks 9.91 to 10.10 ms, no lost ticks, duty 0 throughout, and
the encoders count when a wheel is turned by hand.

### Findings for teammates

- **Buddy 1, `EnableInt`.** Read on core 0 after nav started:
  `NVIC_IPR3 = 0xC0000000`. IRQ15 (core 0's inter-core interrupt) is at 3 as
  expected, but IRQ13 (GPIO) is at 0, not the 2 that `gpio_irq_init()` asks
  for: the byte write to `NVIC_IPR` has no effect on the Cortex-M0+. Fixed by
  branch `buddy2/enableint-word-write`.
- **Buddy 5, dispatcher.** On one boot `PROC1_INTE0` read `0x00C0CCCC`:
  GP0-GP3 (encoders) and GP5 (echo) enabled. The attach race was not seen
  on the car; the fix is on branch `buddy2/gpio-irq-attach-race`.

### Practical notes

- The board cannot run from the USB hub alone: the hub cut the port when
  the motors started. Keep the power bank on whenever USB is connected.
- With the power bank on, BOOTSEL is only read at power-up: power bank off,
  hold BOOTSEL, power bank on, plug in the USB, then let go.
