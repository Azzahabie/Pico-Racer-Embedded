# Buddy 3 line-following and Code 39 audit

## Status

The digital two-sensor line interpreter, edge-based Code 39 A-Z decoder, and
µT-Kernel task integration are implemented. The host decoder test and default
SMP=0 UART firmware build pass. These are software checks only: no robot
wiring, sensor behavior, barcode artwork, timing, or motion has been verified
on hardware.

GPIO numbers below are the current source configuration, not confirmed
physical wiring. Keep motor power disconnected until the wiring, motor
driver, and a safe-stop path have been independently verified. The motor,
IMU, and obstacle task implementations are still TODO stubs, and this work
does not command a motor.

## Implementation

| Stage | Current behavior |
|---|---|
| Input | The line task reads left GP8 and right GP6 as digital inputs (`1 = black`). Barcode GP26 is ADC0 and is sampled independently. |
| Line interpretation | Packed values are `RL`: 00 LOST, 01 left deviation, 10 right deviation, 11 JUNCTION. Black on the left steers right; black on the right steers left. `LINE_STEER_INVERT` flips the steering suggestions; `LINE_STOP_ON_LOST` defaults to 0. |
| Barcode capture | GP26 uses 3000 rising / 1000 falling ADC thresholds. Each transition is timestamped from the RP2040 hardware timer and stored as an alternating bar/space width in microseconds. |
| Decode | The unit-testable decoder assumes Code 39, `*` delimiters, A-Z, and one narrow inter-character gap. It classifies widths from each measured symbol and accepts either scan direction. A successful string is available through `barcode_scanner_get_decoded()`. |
| Console | Application log output uses a shared T-Kernel mutex. Barcode samples are not printed; only completed scan results/errors and line state changes are logged. |
| Navigation | The line task prints suggested commands only. It does not control motors; the motor task remains a stub. |

## Digital line truth table

“Black” means the configured active level (`BLACK_IS_HIGH`) has been observed.
These are software interpretations; validate steering direction on the
stationary robot before connecting motor control.

| GP8 (left) | GP6 (right) | State | Suggested command |
|---:|---:|---|---|
| White | White | LOST | DRIVE by default; STOP only with `LINE_STOP_ON_LOST=1` |
| Black | White | Left deviation | STEER_RIGHT |
| White | Black | Right deviation | STEER_LEFT |
| Black | Black | JUNCTION | DRIVE |

Two digital sensors cannot measure an exact line width or distinguish an
18 mm line from another pattern that covers both sensors. Place both sensors
within the line's 18 mm width when centred, and verify the sensor outputs on
the bench. With `LINE_STOP_ON_LOST=1`, STOP is reported after 40 consecutive
lost samples; this task does not itself stop the motors.

## Viewing the line-reader serial output

The line task prints its startup instructions, changed line states, and a
two-second sensor heartbeat through the shared console logger. The IPC producer, consumer,
and monitor demo tasks are not started, keeping their status output off the
serial console during sensor bring-up.

- **Use the Pico USB cable:** build and flash the `CONSOLE=usb_cdc` profile.
  Set `PICO_SDK_PATH` and initialize the Pico SDK `lib/tinyusb` submodule as
  described in [GETTING_STARTED.md](./GETTING_STARTED.md#1b-pico-sdk---only-if-you-want-usb-cdc-or-wi-fi).
  Open the COM port exposed by the Pico in a serial monitor at **115200 baud**.
  The task waits up to 20 seconds for the USB serial port to connect.
- **Use a USB-to-UART adapter:** build the default `CONSOLE=uart` profile.
  Connect adapter RX to Pico GP0 (UART TX), adapter TX to GP1 (UART RX), and
  connect GND to GND. Open the adapter's COM port at **115200 baud, 8-N-1**.

Opening a serial monitor alone does not select the console transport; it must
match the firmware profile that was built and flashed.

## Code 39 patterns in the decoder

`W` means wide and `N` means narrow, in bar-space order. A letter scan must
contain 29 durations: start `*` (9 elements), a narrow-sized inter-character
space, one letter (9 elements), another narrow-sized space, and stop `*`
(9 elements). The standard Code 39 start/stop pattern is
`NWNNWNWNN`.

For each symbol, the decoder measures the six narrow and three wide elements
and classifies against their midpoint. The minimum wide/narrow ratio is 2.00,
configured by `BARCODE_WIDE_NARROW_MIN_RATIO_X100`. This has not been validated
against the robot's speed or printed artwork.

| Letter | Pattern | Letter | Pattern | Letter | Pattern |
|---|---|---|---|---|---|
| A | `WNNNNWNNW` | B | `NNWNNWNNW` | C | `WNWNNWNNN` |
| D | `NNNNWWNNW` | E | `WNNNWWNNN` | F | `NNWNWWNNN` |
| G | `NNNNNWWNW` | H | `WNNNNWWNN` | I | `NNWNNWWNN` |
| J | `NNNNWWWNN` | K | `WNNNNNNWW` | L | `NNWNNNNWW` |
| M | `WNWNNNNWN` | N | `NNNNWNNWW` | O | `WNNNWNNWN` |
| P | `NNWNWNNWN` | Q | `NNNNNNWWW` | R | `WNNNNNWWN` |
| S | `NNWNNNWWN` | T | `NNNNWNWWN` | U | `WWNNNNNNW` |
| V | `NWWNNNNNW` | W | `WWWNNNNNN` | X | `NWNNWNNNW` |
| Y | `WWNNWNNNN` | Z | `NWWNWNNNN` | | |

These are the standard Code 39 letter patterns in source, not measured
patterns from the course artwork.

The decoder identifies reversed scans by matching the reversed start/stop
delimiter pair, reverses the captured width sequence, and then decodes in
normal Code 39 order. This implementation assumes the stated Code 39
encoding; use a different decoder if the course uses another encoding.

## Commissioning and pin ownership

Set `LINE_COMMISSIONING_MODE` to 1 in `line_config.h` to print GP8/GP6 raw
levels for five seconds over white and then five seconds over black. Move the
stationary sensors between those surfaces when prompted. GP8 and GP6 are
digital-only RP2040 GPIOs and cannot be ADC inputs. GP26 is ADC0; GP28 is
reserved here as battery-sense ADC2.

The configured line-left input GP8 overlaps the stated motor-driver range
GP8-GP11. The firmware deliberately reports this as a pin conflict and does
not choose a replacement pin. Resolve the real wiring before connecting
hardware or enabling motor control.

Producer/consumer affinity is configured in the SMP source: producer on
processor 1 (RP2040 core 0), consumer on processor 2 (core 1). However,
`usermain()` currently leaves creation/start of both tasks and their monitor
commented out, so those fields are not produced by this application. In an
SMP=0 build both would report processor 1. Even when enabled,
`made_on=prc1 seen_on=prc1` alone would not demonstrate cross-core execution.

## Timing and limits

| Setting | Current value | Status |
|---|---:|---|
| Line task delay | 1 ms | Configured kernel tick; actual scheduling jitter is unmeasured. |
| Line input stabilization | 30 samples | Nominally 30 ms, before scheduler jitter. |
| Line-lost debounce | 40 samples | Nominally 40 ms; does not stop unless `LINE_STOP_ON_LOST=1`. |
| Barcode scan end | 500000 us quiet white | Configurable capture timeout; validate against slowest intended speed. |
| Barcode maximum | 24 characters / 259 widths | Fixed storage bound; overlong scans report an error. |

Barcode widths are measured from the RP2040 free-running microsecond timer
when the task observes a threshold crossing. The 1 ms task delay does not
prove that ADC sampling occurs exactly once per millisecond; scheduling jitter
and ADC conversion duration remain to be measured.

## Verification performed

The Code 39 host test is synthetic and exercises:

- All A-Z symbols at multiple width scales, plus A forwards and reversed.

Run the host tests from the repository root:

```sh
gcc -std=c11 -Wall -Wextra -Wpedantic \
  tests/barcode_decode_test.c app_program/line_task/barcode_decode.c \
  -o /tmp/barcode_decode_test
/tmp/barcode_decode_test
```

The full micro T-Kernel default single-core UART target is built with:

```sh
cd build_make
make -j8
```

The host test and firmware build do not test physical sensors, wiring, barcode
artwork, or motion.

## Ordered hardware-validation checklist

1. **Keep motor power disconnected.** Identify the exact robot, controller,
   sensor boards, and revision. Obtain the actual wiring diagram or verify
   every connection by inspection and continuity testing.
2. **Resolve pin ownership.** Reconcile the verified wiring with the
   firmware's GPIO assignments and RP2040/Pico W reservations. Check power,
   ground, voltage levels, pull-ups/downs, and conflicts before connecting
   sensor outputs to the Pico.
3. **Characterize each sensor without motor power.** Measure its digital
   output over the actual track surface, record dark/light levels and noise,
   and set the configured polarity and input pulls from those measurements.
   Confirm sensor spacing, height, alignment, and field of view.
4. **Verify line observations while stationary.** Exercise light, dark, and
   both-sensor conditions; compare the observed logs/state with the truth
   table. Verify both-dark behavior against the course before treating it as
   a physical junction. Tune debounce only from recorded data.
5. **Obtain and measure the real barcode source.** Confirm the assumed Code 39
   A-Z encoding, record narrow/wide elements and inter-character spacing,
   and test both scan orientations. Capture sensor transitions at the intended
   height and speed; validate the 500000 us end-gap against slow scans.
6. **Measure timing.** Measure sampler cadence, task execution time, and
   scheduling jitter on hardware with reviewed instrumentation on a
   confirmed spare pin or a suitable trace method. Do not assume the
   commented profiling-pin setting performs this measurement.
7. **Define the missing navigation contract.** Agree on sensor-to-motion
   semantics, junction handling, letter-to-course mapping, motor interface,
   and error behavior with the responsible team. Implement and test a
   fail-safe stop path before connecting motor power.
8. **Only after the electrical and safety gates above pass**, perform any
   future motion test under the project's approved safety procedure, with a
   verified motor driver, accessible stop mechanism, and controlled test
   area. This code currently has no motor/navigation implementation, so a
   powered motion test is not part of the present verification.

## Unresolved blockers

- Physical pin wiring, sensor model/polarity, spacing, height, and output
  levels are unverified; source GPIO values are provisional.
- The actual barcode artwork and six measured course patterns are
  unavailable, so neither artwork geometry nor physical decodability can be
  verified.
- Sampling cadence, scheduler jitter, barcode duration limits, sensor noise,
  and speed margins have not been measured on the robot.
- Two line sensors cannot distinguish both-dark junctions from all other
  both-dark shapes.
- No course map, letter-to-command mapping, navigation consumer, motor
  control interface, or safety-stop path exists in this implementation.
- The Code 39 A-Z assumption and the quiet-gap timeout need confirmation against
  the actual barcode format, artwork, and scanning speed.
