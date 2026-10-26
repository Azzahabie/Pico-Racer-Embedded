# Buddy 3 line-following and Code 39 audit

## Status

The digital two-sensor line interpreter, single-letter Code 39 A-Z decoder,
edge capture, bounded record transport, and µT-Kernel task integration are
implemented. Host tests and both firmware build targets pass. These results
are software checks only: no robot wiring, sensor behavior, barcode artwork,
timing, or motion has been verified on hardware.

GPIO numbers below are the current source configuration, not confirmed
physical wiring. Keep motor power disconnected until the wiring, motor
driver, and a safe-stop path have been independently verified. The motor,
IMU, and obstacle task implementations are still TODO stubs, and this work
does not command a motor.

## Implementation

| Stage | Current behavior |
|---|---|
| Input | The stand-alone `line_task` reads the left digital line sensor on GP8 and the right sensor on GP6. `BLACK_IS_HIGH` selects the assumed active level; confirm it with the actual sensor before running the car. |
| Line interpretation | `line_logic` normalizes the configured electrical polarity, accepts steering-state changes after three samples, and accepts line-lost after eight consecutive samples (nominally 40 ms at the 5 ms sample delay). |
| Barcode capture | `bar_capture` debounces transitions, measures alternating bar/space durations in sampler ticks, ends a scan after a quiet gap, and aborts on overlong elements or failed record delivery. |
| RTOS transport | The stand-alone line test reports directly to the monitor; it does not send sensor records to another task. |
| Decode | `barcode_decode` recognizes standard Code 39 start/stop framing and the letters A-Z. A decoded letter is reported as **unmapped**; no route or motor action is inferred. |
| Navigation | The stand-alone line task prints suggested DRIVE/STEER/STOP commands only. It does not control motors; there is no navigation or motor-control consumer. |

The message buffer is configured for 64 records with 8 bytes of kernel buffer
accounting per record. The separate bring-up SPSC ring uses an empty slot to
distinguish full from empty, so its effective capacity is 63 records.

## Digital line truth table

“Black” means the configured active level (`BLACK_IS_HIGH`) has been observed.
These are software interpretations; validate steering direction on the
stationary robot before connecting motor control.

| GP8 (left) | GP6 (right) | State | Suggested command |
|---:|---:|---|---|
| Black | Black | Centred | DRIVE |
| Black | White | Line under left sensor | STEER_LEFT |
| White | Black | Line under right sensor | STEER_RIGHT |
| White | White | Line lost | STOP |

Two digital sensors cannot measure an exact line width or distinguish an
18 mm line from another pattern that covers both sensors. Place both sensors
within the line's 18 mm width when centred, and verify the sensor outputs on
the bench. The STOP suggestion is reported after eight consecutive lost
samples; this task does not itself stop the motors.

## Viewing the line-reader serial output

The line task prints its startup instructions, changed line states, and a
one-second sensor heartbeat through `tm_printf`.

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
contain 29 durations: start `*` (9 elements), one narrow inter-character
space, one letter (9 elements), another narrow inter-character space, and
stop `*` (9 elements). The standard Code 39 start/stop pattern is
`NWNNWNWNN`.

The decoder derives a narrow/wide threshold from the known start frame, which
has three wide elements, and requires the measured wide/narrow separation to
meet `B3_CODE39_WIDE_RATIO_X100` (2.00 by default). This calibration and
threshold are not validated against the robot's sensor, speed, or printed
artwork.

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

Direction is another unresolved input. Some single-letter patterns can be
valid in both orientations as different letters. For example, reversing
some valid symbols collides with another valid symbol. Without a known scan
direction, direction signal, or asymmetric marker, a decoder cannot both
accept all letters and guarantee rejection of every reversed scan.

## Timing and limits

| Setting | Current value | Status |
|---|---:|---|
| Line task delay | 1 kernel tick | Relative `tk_dly_tsk`; the RP2040 kernel tick is configured as 1 ms, but actual sampling interval and scheduler jitter are unmeasured. |
| Line input stabilization | 2 samples | Software setting; not measured against sensor noise or robot speed. |
| Junction candidate debounce | 3 samples | Software setting; “both dark” is not proof of a physical junction. |
| Barcode transition stabilization | 2 samples | Software setting; sensor waveform has not been measured. |
| Longest accepted barcode element | 400 sampler ticks | Provisional; requires sensor, speed, and artwork measurements. |
| Quiet-gap scan end | 500 sampler ticks | Provisional; requires measurement and timing validation. |
| Maximum assembled durations | 32 | Software bound; valid single-letter frame currently uses 29 durations. |

Ticks in barcode capture are counts of sampler calls, not measured
milliseconds. A 1 ms kernel tick setting does not prove the line task runs
exactly once per millisecond. The commented `B3_PROFILE_PIN` definition alone
does not instrument or measure task timing.

## Verification performed

The host test suite is synthetic and exercises:

- all 26 letter patterns, width scaling/noise, malformed and reversed frames;
- assembler abort, recovery, and overflow behavior;
- every digital line truth-table combination under both polarities;
- junction debounce/rearm, ring saturation, and capture timeout/recovery.

Run the host tests from the repository root:

```sh
gcc -std=c11 -Wall -Wextra -Wpedantic \
  tests/buddy3_host_test.c \
  app_program/line_task/barcode_decode.c \
  app_program/line_task/bar_capture.c \
  app_program/line_task/bar_ring.c \
  app_program/line_task/line_logic.c \
  -o /tmp/buddy3_host_test
/tmp/buddy3_host_test
```

All eight test groups passed with Ubuntu GCC 15.2.0.

The full micro T-Kernel default single-core UART target builds with:

```sh
cd build_make
make -j8
```

This task's full build passed in a writable WSL copy with Ubuntu
`arm-none-eabi-gcc` 14.2.1, producing `mtk3pico_smp0_uart.elf` and
`mtk3pico_smp0_uart.uf2`. Existing generated output directories in the
Windows-mounted workspace were not writable from WSL, so the build was run
in a separate writable copy. This is a successful build with that toolchain,
not a qualification against the repository's pinned 13.2.1 baseline.

The standalone Pico SDK barcode bring-up target also configured and built
successfully with the installed Pico SDK 1.5.1 and ARM GCC 10.3.1. It is a
separate bring-up image, not the micro T-Kernel firmware target. Neither
build flashes or tests a physical robot.

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
   table. Determine whether both-dark is a junction on the real course or
   merely a wide/centered line. Tune debounce only from recorded data.
5. **Obtain and measure the real barcode source.** Restore the artwork and
   six intended samples, record their exact narrow/wide elements and
   inter-character spacing, and test both scan orientations. Capture sensor
   transitions at the intended height and speed; tune the 400/500 tick
   limits only from those measured waveforms.
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
- Universal reversed-scan rejection is ambiguous for single Code 39 letters
  without additional direction information.
