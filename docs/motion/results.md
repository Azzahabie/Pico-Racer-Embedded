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
| Counts per wheel revolution | 2535 | 2533 | 10 slow hand turns against a tyre mark, change in count / 10, 1 trial each (A2 asks for 3) |
| Count when the wheel turns forward, before flags | up | down | hand turn |
| Missed transitions while turning by hand | 0 | 13 in 25,334 | `missed` in the status line |

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
