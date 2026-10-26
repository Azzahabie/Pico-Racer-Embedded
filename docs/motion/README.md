# Motion control (Buddy 2): bring-up guide

`motion_task` owns the two motors and the two wheel encoders. It proves the
hardware on the bench: PWM on the H-bridge, both encoders counting the right
way, the 10 ms tick on core 1, the FR-30 stall check and the hardware
watchdog. It also handles Buddy 5's urgent
`STOP_SCAN` request and brakes both motors. The command interface from
`nav_ctrl_task`, obstacle maneuver commands and `STOP_IMU` are still pending.

## What is in the code

| File | What it does |
|---|---|
| `app_program/motor_task/motor_task.c/.h` | `motion_task` (core 1, priority 11, 10 ms tick from a cyclic handler through an event flag) and `motion_report` (core 0, priority 24, does all the printing). Bench test, stall check, timing measurement. |
| `app_program/motor_task/motor.c/.h` | PWM driver: GP8/GP9 = slice 4, GP10/GP11 = slice 5, 10 kHz. One 32-bit store updates both inputs of a motor. |
| `app_program/motor_task/encoder.c/.h` | Quadrature (x4) on GP0..GP3 with pull-ups. Both counts published as one packed 32-bit word. |
| `app_program/motor_task/motion_lib.c/.h` | Pure arithmetic, no kernel calls: quadrature table, packed counts, wrap-safe delta, stall detector, integer PID with anti-windup. |
| `app_program/motor_task/motion_wdt.c/.h` | RP2040 hardware watchdog: armed at start-up, fed at the end of every completed tick, and the reason for the last reset. |
| `app_program/motor_task/motion_cfg.h` | Every pin, flag and tunable in one place. |
| `app_program/common/gpio_irq.c/.h` | Buddy 5's shared GPIO dispatcher, the one owner of the GPIO bank interrupt. Motion attaches GP0..GP3 one pin at a time on core 1 (see "For the team"). |
| `tests/motion_lib/` | Host unit tests for `motion_lib` (UBSan/ASan). |
| `tests/encoder/` | The encoder driver against a fake dispatcher with Buddy 5's interface. |
| `tests/encoder_dispatch/` | The encoder driver through Buddy 5's real `gpio_irq.c` on fake registers, both cores. |
| `tests/motion_wdt/` | The watchdog driver against fake registers: the arming steps and their order, LOAD, the feed, the reset cause. |
| `tests/motion_task_sim/` | The real `motor_task.c` run on a PC against a simulated car, with the bench off and on, and the watchdog test image. |

Run the host tests in the build container (on macOS the host compiler's
AddressSanitizer aborts at start-up):

```sh
docker compose run --rm build bash -lc "make -C tests"   # all suites
make -C tests/motion_task_sim V=1         # (in the container) show the simulated console
make -C tests/encoder_dispatch race       # (in the container) the open dispatcher race, see "For the team"
```

Layout follows Barr-C 2018 (spaces not tabs, 4-space indent, 80 columns,
constant on the left of `==`, `for (;;)`, ISR names end `_isr`) through
`app_program/motor_task/.clang-format`.

## Build and flash

**Packed or unpacked (Week 13 evidence).** `MOTION_PACKED` in
`motion_cfg.h` is committed as 1: the packed layouts. 0 builds the Week 6
(unpacked) layouts on the same code, only to measure the Week 13 "before".
Today it switches only the encoder counts (one packed word against two
32-bit counters copied with interrupts off). `sh tests/week13/measure.sh`
builds HEAD both ways and prints motion's RAM; `make -C tests` tests both.
Every ready line ends with `MOTION_PACKED=1` or `=0`, so keep it in each
timing log. Method and numbers: `week13/baseline.md`.

**Bench image or integration image.** The bench test below is built only
when `MOTION_BENCH` is 1 in `motion_cfg.h`. The committed value is 0: that
build never reads or configures GP20 (Buddy 5's start button), nothing can
start the motors, and the ready line ends with
`bench OFF (MOTION_BENCH=0): motors stay braked`. For the bench procedure,
set it to 1 locally, rebuild, and set it back to 0 before committing.
Both images have the same file name, so check the ready line before the
car ever goes on the floor. In a bench image GP20 is also nav_ctrl_task's
start button, so one press starts both the bench test and nav's mission
(scan_task then runs its scans); another reason bench images are wheels-off
only.

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

In a bench image, the motors stay braked until you press the button on
**GP20**. Pressing it again aborts. Any fault brakes both motors.

## Bench procedure

The output lines below show the format (numbers from the host simulation);
your car's numbers will differ.

### Step 0: boot line

```
[motion] ready on processor 2, init OK (er=0). PWM 10000 Hz on GP8-GP11, encoders GP0-GP3, button GP20, test=open-loop, MOTION_PACKED=1
[motion] last reset: power-on or RUN pin.
[motion] watchdog armed: the chip resets if a tick does not complete within 30 ms.
```

`processor 2` = core 1. `init FAILED` means the motors are held braked and
nothing else runs: report the `er` value. The watchdog is then not armed,
so the car does not reboot over and over.

`last reset` says why the chip started:

| Line | Meaning |
|---|---|
| `power-on or RUN pin.` | Normal start. |
| `a reboot by the boot ROM or a debugger (such as after a UF2 copy), not a fault.` | Normal after flashing without switching off. |
| `WATCHDOG. motion_task stopped feeding it ...` | motion_task hung, or its tick stopped, for 30 ms, and the chip reset. The motors stay braked until a command (a button press in a bench image). Report it with the log before it. |

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
- `missed` counts illegal transitions (both channels changed at once). It
  is a lower bound: an interrupt held off for three or more transitions
  can decode as a smaller step without counting as missed.
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
[motion] watchdog (5 s): longest gap between feeds 10000 us, resets at 30000 us
```

This is the core 1 timing evidence: tick jitter, the longest tick, the
encoder interrupt rate and its longest run, and the longest gap between
two watchdog feeds. The 30 ms watchdog is safe only while that gap stays
well below 30000 us; record the worst one, with Wi-Fi on too once motion
runs in the Wi-Fi build.

- `encoder irq N/s` counts GPIO interrupts that reached the encoder
  callback. Callbacks from one interrupt share the dispatcher's entry
  timestamp, so two pins changing together count once.
- `longest` runs from the dispatcher's timer read (the first thing its
  handler does) to the end of the encoder callback. It is a lower bound:
  it leaves out the exception entry, the kernel's interrupt wrapper and any
  later callback in the same interrupt, such as Buddy 5's ECHO pin.

### Watchdog (PreTri1 row 2, PostTri1 row 9)

`motion_task` arms the RP2040 hardware watchdog once its set-up has passed
and feeds it as the last thing in every completed tick. If the task hangs,
or its tick stops (core 0 stalled, kernel timer stopped, a fault handler
spinning), the chip resets 30 ms after the last feed (`MOTION_WDT_MS`,
proposed). The reset puts GP8 to GP11 back to inputs with pull-downs, so
the driver sees both inputs low (brake). After the reboot the motors stay
braked until a command.

**Test 1, feed gap** (any image): the `watchdog (5 s)` line above.

**Test 2, a hang on purpose** (wheels off). Build a bench image with
`MOTION_BENCH` 1 and `MOTION_TEST_WDT` 1 (locally, never committed) and
flash it. Press the button: 0.5 s into `L fwd`, with the left wheel
turning, motion_task stops feeding the watchdog and spins. Expect:

- the left wheel stops and the USB port drops (the chip reset);
- the port comes back and the boot lines say:

```
[motion] last reset: WATCHDOG. motion_task stopped feeding it (task hung or tick stopped). Motors stay braked until a command.
[motion] watchdog armed: the chip resets if a tick does not complete within 30 ms.
[motion] watchdog test: the reset came NNNNN us after the last feed (set to 30000 us).
```

`NNNNN` is measured: the hang writes the 1 MHz time to a watchdog scratch
register until the reset, and the next boot subtracts the time of the last
feed. It should be about 30000. About 15000 or 60000 would mean the LOAD
doubling (erratum RP2040-E1) is wrong for this chip.

**Test 3, pins during a reset** (meter, no image needed). Car on battery,
hold the Robo Pico's RESET button (it holds RUN low) and measure GP8, GP9,
GP10 and GP11 to GND. All four should read about 0 V (brake).

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
| Watchdog: longest gap between feeds (us) | | |
| Watchdog test: reset after the last feed (us), wheel stopped (yes/no) | | |
| GP8 / GP9 / GP10 / GP11 with RESET held (V) | | |

## For the team

Found while writing this; each needs its owner.

1. **GP0/GP1 UART mirror (port).** The USB-CDC build kept UART0 on GP0/GP1
   as a mirror, but those are the left encoder pins. Commit "port: no
   UART0 mirror on GP0/GP1 in USB-CDC builds" turns the mirror off in that
   profile only; the UART profile is unchanged. Without it the Pico drives
   GP0 against the encoder from boot, and every `tm_printf` waits about
   87 us for each character past the 32-byte UART FIFO, with interrupts
   masked on that core.
2. **One GPIO interrupt for the whole chip (Buddy 5).** All GPIO edge
   interrupts are one interrupt number with one vector slot, shared by both
   cores; a second `tk_def_int(13, ...)` silently replaces the first. The
   car uses Buddy 5's dispatcher in `app_program/common/gpio_irq.h`:
   `gpio_irq_init()` on each core, then `gpio_irq_attach()` per pin.
   Open proposal: `gpio_irq_attach()` updates `PROCx_INTE` with a
   read-modify-write while interrupts are on. If scan_task (GP5) is
   preempted there by motion_task attaching GP0..GP3 on the same core, the
   encoder enables are lost; `make -C tests/encoder_dispatch race` shows it.
   Writing the bits with `clr_w()` then `set_w()` (the RP2040's atomic
   clear and set aliases) fixes it, and the race test then passes.
3. **Print from core 0 only.** In the USB-CDC build `tm_printf` masks
   interrupts on its core for the whole line. Any task that prints should
   be pinned to core 0 (`TP_PRC1`), or the encoder interrupt on core 1 is
   delayed. scan_task prints on core 1. The timing line cannot show that
   delay (its `longest` starts only once interrupts are unmasked); the
   evidence is the status line's `missed L/R` while the wheels turn with
   scan_task running, where any non-zero value means an edge came late.
4. **GP16 (Buddy 4).** `BOARD_LED_PIN` is 16: `hw_setting.c` makes it an
   output at boot and `blink_task` (`TM_BLINK_LED=1` only) toggles it, but
   GP16 is the IMU's Grove 4 pin.
5. **GP8/GP9 I2C (Buddy 4).** `hw_setting.c` (sample driver,
   `USE_SDEV_DRV`) puts I2C0 on GP8/GP9, which are the left motor pins.
   `motor_init()` takes them back for PWM at start-up; the IMU's I2C needs
   to move to GP16/GP17.
6. **led_test** drove GP10 (right motor M2A). Buddy 1 removed it on
   master (21 Oct).
7. **Watchdog (everyone, once motion runs in the shared build).**
   motion_task arms the RP2040 hardware watchdog with 30 ms and feeds it
   every 10 ms tick, and the tick comes from core 0's kernel timer. So if
   anything holds off core 0's timer interrupt for about 20 ms (interrupts
   masked, a long critical section), the whole car resets. The timing
   line's feed gap will show it. Only motion_wdt.c writes the watchdog
   registers; cpu_clock.c sets its TICK once at boot, which must stay.
8. **Debug flags (Buddy 1).** With `BOARD_DIAG_GPIO_SCAN=1` and
   `TM_BLINK_LED=1`, blink's diagnostic pulse (`demo_tasks.c`) takes
   GP8-GP11 and GP2/GP3 back after motion has set them up. Keep both off
   on the car.

## Not done yet

- `STOP_SCAN` is wired from scan_task to motion_task, but its response time
  has not yet been measured on the car. The avoid-command buffer is not
  created or bound, so bypass/reverse commands and their replies are simulated.
- Command interface from `nav_ctrl_task` (CMD bit), `STOP_IMU`,
  Motion Fault exit on FAULT_CLEAR(MOTION).
- Distance, straight-line correction, encoder turns.
- Minimum-bit command and telemetry formats, once `common/messages.h` is
  agreed with Buddy 1. The 52-byte print record here is motion's console
  channel, in every build and the same in both MOTION_PACKED settings; it
  is not the Week 13 "before" (see week13/baseline.md).
- Watchdog tests 2 and 3 on the car, and the feed gap with Wi-Fi on.
