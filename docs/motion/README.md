# Motion control (Buddy 2): bring-up guide

`motion_task` owns the two motors and the two wheel encoders. This first
version proves the hardware on the bench: PWM on the H-bridge, both
encoders counting the right way, the 10 ms tick on core 1, and the FR-30
stall check. The command interface from `nav_ctrl_task`, STOP_SCAN /
STOP_IMU and the hardware watchdog come next.

## What is in the code

| File | What it does |
|---|---|
| `app_program/motor_task/motor_task.c/.h` | `motion_task` (core 1, priority 11, 10 ms tick from a cyclic handler through an event flag) and `motion_report` (core 0, priority 24, does all the printing). Bench test, stall check, timing measurement. |
| `app_program/motor_task/motor.c/.h` | PWM driver: GP8/GP9 = slice 4, GP10/GP11 = slice 5, 10 kHz. One 32-bit store updates both inputs of a motor. |
| `app_program/motor_task/encoder.c/.h` | Quadrature (x4) on GP0..GP3 with pull-ups. Both counts published as one packed 32-bit word. |
| `app_program/motor_task/motion_lib.c/.h` | Pure arithmetic, no kernel calls: quadrature table, packed counts, wrap-safe delta, stall detector, integer PID with anti-windup. |
| `app_program/motor_task/motion_cfg.h` | Every pin, flag and tunable in one place. |
| `app_program/common/gpio_irq.c/.h` | The one owner of the GPIO bank interrupt (see "For the team"). |
| `tests/motion_lib/` | Host unit tests for `motion_lib` (UBSan/ASan). |
| `tests/motion_task_sim/` | The real `motor_task.c` run on a PC against a simulated car. |

Run the host tests (plain `gcc`, no Docker, no board):

```sh
make -C tests                         # all suites
make -C tests/motion_task_sim V=1     # also show the simulated console
```

Layout follows Barr-C 2018 (spaces not tabs, 4-space indent, 80 columns,
constant on the left of `==`, `for (;;)`, ISR names end `_isr`) through
`app_program/motor_task/.clang-format`.

## Build and flash

The car uses the USB console (approved pin map: GP0/GP1 are the left
encoder), so build the USB-CDC profile:

```sh
docker compose run --rm build bash -lc "cd build_make && make SMP=1 CONSOLE=usb_cdc -j8"
```

Output: `build_make/mtk3pico_smp1_usb_cdc.uf2`.

1. Car switched off, **wheels off the ground** (box under the chassis).
2. Hold BOOTSEL on the Pico W, plug the USB cable into the laptop, release.
3. Copy the `.uf2` onto the `RPI-RP2` drive. The Pico reboots by itself.
4. Open the serial port: Windows `COMx` in PuTTY or the VS Code Serial
   Monitor, macOS `/dev/tty.usbmodem*`, Linux `/dev/ttyACM0`
   (`screen /dev/ttyACM0 115200`; the baud rate does not matter for USB).
5. Switch on the car's battery so the motors have power.

Motors stay braked until you press the button on **GP20**. Pressing it
again aborts. Any fault brakes both motors.

## Bench procedure

The output lines below show the format (numbers from the host simulation);
your car's numbers will differ.

### Step 0: boot line

```
[motion] ready on processor 2, init OK (er=0). PWM 10000 Hz on GP8-GP11, encoders GP0-GP3, button GP20, test=open-loop
```

`processor 2` = core 1. `init FAILED` means the motors are held braked and
nothing else runs: report the `er` value.

### Step 1: turn each wheel by hand (motors braked)

Turn the **left** wheel slowly in the car's forward direction:

```
[motion] t=12.40s  duty L+0 R+0  count L+117 R+0  speed L+585 R+0 c/s  missed L0 R0
```

- L should go **up** and R should stay at 0. If R moves, the encoder plugs
  are swapped. If L goes down, set `MOTION_ENC_L_INVERT` to 1.
- Mark the tyre, turn exactly **one revolution**, and note the change in
  count: that is counts per wheel revolution (assumption A2).
- Repeat for the right wheel.
- `missed` counts illegal transitions (both channels changed at once).
  It should stay at 0 when turning by hand.

The braked H-bridge makes the wheel stiff; turning slowly is fine.

### Step 2: open-loop test (press GP20)

Each wheel runs forward then backward at 35 % duty for 1.5 s, then both
forward, with 1 s braked between steps. Watch the wheels while it runs.

```
[motion] STEP L fwd    duty L+350 R+0 -> moved L+5060 R+0 counts (L+3373 R+0 c/s, L+33 R+0 per tick)
```

Set the flags in `motion_cfg.h` from what you see on each `fwd` step:

| Wheel turned | Count | Set |
|---|---|---|
| forward | up | nothing |
| forward | down | `MOTION_ENC_x_INVERT 1` |
| backward | down | `MOTION_MOTOR_x_INVERT 1` |
| backward | up | both `MOTION_MOTOR_x_INVERT 1` and `MOTION_ENC_x_INVERT 1` |

Rebuild, reflash and repeat until every `fwd` step turns the wheel forward
with the count going up. The test also warns when a one-wheel step moves
the other encoder (plugs swapped) and when a driven wheel counts 0.

### Step 3: dead band (runs automatically after step 2)

```
[motion] DEAD BAND: left starts at 150, right starts at 175 per-mille (0 = did not move by 600)
```

Put the larger number (plus a margin) into `MOTION_DEADBAND_DUTY`.

### Step 4: closed-loop speed (after steps 1 to 3 pass)

Set `MOTION_PID_TARGET` to about half of the "per tick" number from the
`both fwd` step, set `MOTION_TEST_PID` to 1, rebuild, reflash, press GP20.
Both wheels hold the target for 3 s; the status lines show the measured
speed (`c/s` = target x 100 when it is on target). A wrong INVERT flag
ends as a stall fault, not a runaway.

### Stall check (FR-30)

The check judges **net** movement: a driven wheel must move at least 3
counts in one direction within 300 ms (`MOTION_STALL_TICKS`,
`MOTION_STALL_MIN_COUNTS`, both still proposed values). It also counts the
edges on each encoder phase, which separates the two FR-30 faults:

| What happened | Fault |
|---|---|
| No edges at all (wheel blocked, or the whole encoder dead) | `wheel stalled` (motor stall) |
| One phase keeps switching, the other never does, count goes nowhere (one signal wire unplugged) | `encoder fault` |

A fully dead encoder (power or ground unplugged) shows up as a motor stall:
without current sensing the board cannot tell it from a blocked wheel.

Each phase's edges are counted only since the other phase last moved, so a
wire that comes off while the wheel is turning is still reported as an
encoder fault. Known limit: a held wheel whose encoder rests exactly on a
phase edge and chatters on that one phase also reads as an encoder fault.
`MOTION_STALL_ENC_EDGES` (proposed 3) is the number of one-phase edges
needed; if test 1 below ever reports `encoder fault`, raise it above the
chatter you see and note it in the results table.

**Test 1, wheel held.** Hold one wheel still with your hand during a `fwd`
step (gently). Within about 300 ms:

```
[motion] FAULT 0x01: left wheel stalled  motors braked. Fix, then press the button to run again.
```

**Test 2, one wire unplugged** (FR-30's own test). Switch off, unplug only
the A wire (or only the B wire) of one encoder at the jumper, switch on,
press the button. On that wheel's first `fwd` step, within about 300 ms:

```
[motion] FAULT 0x10: left encoder fault  motors braked. Fix, then press the button to run again.
[motion]   encoder fault: one phase kept switching but the count went nowhere. Check the A and B wires of that encoder.
```

(0x10 is the left encoder fault bit, 0x20 the right one.) Repeat with the
encoder's power wire unplugged: that one reports `wheel stalled`.

### Timing evidence (every 5 s)

```
[motion] timing (5 s): tick period 9998..10003 us, tick work max 41 us, encoder irq 4210/s (longest 3 us), lost ticks 0, dropped reports 0
```

This is the core 1 timing evidence: tick jitter, the longest tick, the
encoder interrupt rate and its longest run.

## Record the results

| Item | Left | Right |
|---|---|---|
| Counts per wheel revolution (A2) | | |
| `MOTION_MOTOR_x_INVERT` | | |
| `MOTION_ENC_x_INVERT` | | |
| Dead band (per-mille) | | |
| Speed at 350 per-mille (counts per tick) | | |
| Stall fault time with the wheel held (ms), and which fault | | |
| Encoder fault with the A wire unplugged (yes/no, ms) | | |
| Encoder fault with the B wire unplugged (yes/no, ms) | | |
| Fault with the encoder power wire unplugged (which one) | | |
| Tick period min..max (us), work max (us) | | |
| Encoder interrupt longest (us), rate at 350 (per s) | | |

## For the team

Found while writing this; each needs its owner.

1. **GP0/GP1 UART mirror (port).** The USB-CDC build kept UART0 on GP0/GP1
   as a mirror, but those are the left encoder pins. Commit "port: no
   UART0 mirror on GP0/GP1 in USB-CDC builds" turns the mirror off in that
   profile only; the UART profile is unchanged. Without it the Pico drives
   GP0 against the encoder from boot, and every `tm_printf` waits about
   87 us for each character past the 32-byte UART FIFO, with interrupts
   masked on that core.
2. **One GPIO interrupt for the whole chip.** All GPIO edge interrupts are
   one interrupt number with one vector slot, shared by both cores. A second
   `tk_def_int(13, ...)` silently replaces the first. Anyone who needs a
   GPIO edge interrupt (Buddy 5's echo pin, a start button on core 0) calls
   `gpio_irq_init()` once on their core and then `gpio_irq_attach()`, from
   `app_program/common/gpio_irq.h`.
3. **Print from core 0 only.** In the USB-CDC build `tm_printf` masks
   interrupts on its core for the whole line. Any task that prints should
   be pinned to core 0 (`TP_PRC1`), or the encoder interrupt on core 1 is
   delayed. The demo `monitor` and `blink` tasks are not pinned.
4. **GP16 (Buddy 4).** `BOARD_LED_PIN` is 16 and the demo `blink` task
   toggles it, but GP16 is the IMU's Grove 4 pin.
5. **GP8/GP9 I2C (Buddy 4).** `hw_setting.c` (sample driver,
   `USE_SDEV_DRV`) puts I2C0 on GP8/GP9, which are the left motor pins.
   `motor_init()` takes them back for PWM at start-up; the IMU's I2C needs
   to move to GP16/GP17.
6. **led_test** drove GP10 (right motor M2A). It is no longer started.

## Not done yet

- Command interface from `nav_ctrl_task` (CMD bit), STOP_SCAN / STOP_IMU,
  Motion Fault exit on FAULT_CLEAR(MOTION).
- RP2040 hardware watchdog fed each tick (proposed 30 ms).
- Distance, straight-line correction, encoder turns.
- Minimum-bit command and telemetry formats, once `common/messages.h` is
  agreed with Buddy 1. The 52-byte report record here is the unpacked
  "before" baseline.
- `motion_task` waits for nothing but its event flag, but this version
  has not been timed on the car yet: fill the table above.
