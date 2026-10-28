# Buddy 3 line-following and Code 39 audit

## Status

The line task reads the left digital sensor on GP8 and right digital sensor
on GP6. It captures GP26 ADC edges, measures widths with the RP2040 hardware
timer, and attempts to decode Code 39 A-Z. These are software behaviors only;
physical wiring, barcode artwork, timing margins, and sensor response have
not been verified on hardware.

All line/barcode/battery pins, ADC thresholds, and compile-time behavior flags
are in `app_program/common/robot_config.h`. That configuration emits a
compiler warning because GP8 is assigned to both the left line sensor and
the motor-driver range GP8-GP11. GP28 is reserved for battery sense (ADC2).
GP6 and GP8 are digital GPIOs, not ADC-capable pins. This task only prints
suggested commands; it does not command a motor.

## Implementation

| Stage | Current behavior |
|---|---|
| Input | Left line sensor GP8, right line sensor GP6 (digital); barcode GP26/ADC0 (analog). |
| Line interpretation | Bits are 0bRL, with 1=black. A new line state is accepted after six samples; 00 is accepted after eight samples. |
| Barcode capture | GP26 uses 3000 rising and 1000 falling ADC thresholds. Edge widths use hardware-timer microseconds; a valid stop delimiter completes immediately. Invalid scans end after white idle of max(50,000 us, 4x the narrowest observed run). |
| Decode | Assumes standard Code 39, `*` delimiters, 9 elements per symbol, inter-character spaces, and A-Z data. Forward and reversed scans are attempted. |
| Output | One decoded/error line is printed per scan. The latest decoded alphabet is also available through `barcode_scanner_copy_decoded()`. |

## Digital line truth table

“Black” means the sensor's configured active level. The steering command
follows the requested geometry; `LINE_STEER_INVERT=1` flips it at build time.

| GP8 left | GP6 right | State | Command |
|---|---|---|---|
| White | White | 00 straight | DRIVE (`LINE_STOP_ON_LOST=1` changes this to STOP) |
| Black | White | 01 left deviation | STEER_RIGHT |
| White | Black | 10 right deviation | STEER_LEFT |
| Black | Black | 11 junction | DRIVE |

Two digital sensors cannot measure an exact line width or distinguish an
18 mm line from another pattern that covers both sensors. This task reports
commands only and does not stop or drive the motors.

## Viewing the line-reader serial output

The line task prints its startup instructions, changed line states, and a
two-second sensor heartbeat through `tm_printf`. The IPC producer, consumer,
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

`W` means wide and `N` means narrow, in bar-space order. A one-letter scan
contains 29 measured widths: start `*` (9 elements), an inter-character
space, the letter (9 elements), another inter-character space, and stop `*`
(9 elements). Each symbol's narrow/wide split is inferred from its measured
widths, so the classification scales with scan speed. The standard
Code 39 start/stop pattern is `NWNNWNWNN`.

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

These are the static standard Code 39 letter patterns in source, not six
measured patterns from the course. The repository does not contain the
`BARCODE.pptx` artwork, and the patterns in the supplied attachment were
flattened together; the six physical samples therefore cannot be
independently reconstructed or checked.

The decoder tries both element orientations and reports `dir=fwd` or
`dir=rev`. A reverse scan's decoded text is restored to the original character
order. Use the `dir` field when scan direction matters to the application.

## Timing and limits

| Setting | Current value | Status |
|---|---:|---|
| Task delay | 1 kernel tick | ADC sampling cadence depends on scheduling and hardware; timer values are in microseconds. |
| ADC thresholds | rising 3000, falling 1000 | Based on reported black (~4045) and white (~135) readings. |
| Invalid-scan idle end | max(50,000 us, 4x narrowest run) | Configurable baseline in `robot_config.h`; total scan limit is 10 s. |
| Width buffer | 512 runs | Static capture limit; decoded text is capped at 48 letters. |

## Verification performed

The unit-testable `barcode_decode_widths()` function is covered by a
standalone host test for forward/reversed `A` and `ABC` scans, varying widths,
and bad-length rejection. Run it with:

```sh
gcc -std=c11 -Wall -Wextra -Wpedantic \
  tests/barcode_scanner_decode_test.c \
  app_program/line_task/barcode_scanner.c \
  -o barcode_scanner_decode_test
./barcode_scanner_decode_test
```

The firmware build command is:

```sh
docker compose run --rm build bash -lc "cd build_make && make -j8"
```

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
   and confirm the configured polarity and thresholds against those measurements.
   Confirm sensor spacing, height, alignment, and field of view.
4. **Verify line observations while stationary.** Exercise light, dark, and
   both-sensor conditions; compare the observed logs/state with the truth
   table. Determine whether both-dark is a junction on the real course or
   merely a wide/centered line. Tune debounce only from recorded data.
5. **Obtain and measure the real barcode source.** Restore the artwork and
   six intended samples, record their exact narrow/wide elements and
   inter-character spacing, and test both scan orientations. Capture sensor
   transitions at the intended height and speed; validate width classification
   and the configured quiet-gap timeout from those waveforms.
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
- Code 39 is an unconfirmed encoding assumption. Replace the decoder if the
  actual barcode format differs.
