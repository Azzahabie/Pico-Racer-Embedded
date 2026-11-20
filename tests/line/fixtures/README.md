# Analog barcode traces (car, 20 Nov 2026)

Readings of the analog IR barcode module on GP28 (ADC2) from the lab test
image of 20 Nov (`lab_analog` / `lab_analog2_relative`, not committed),
used by `tests/line/test_bc_analog_fixtures.c`. Measured on the car (M);
how they were rebuilt is below, and that part is a reconstruction.

| File | Car log, time | What happened on the car |
|---|---|---|
| `run3_1509_decoded_Z.json` | `serial_20261119_222026_S1.log`, 15:09:45-15:10:03 | the test image decoded `Z`, the armed junction fired |
| `run1_1506_Z_missing_first4.json` | same log, 15:06:04-15:06:30 | `*Z*`, but the first narrow bar, the wide space, the second narrow bar and the narrow space never rose above the trace level |
| `run4_1517_track_faint.json` | `serial.log` (Wi-Fi netlog), 15:17:26-15:17:43 | the track's sheet, faint thin bars (500-850 counts) |
| `run2_1508_faint_aborted.json` | `serial_20261119_222026_S1.log`, 15:08:11-15:08:39 | faint thin bars, read aborted `width_too_long` |

Each file holds:

- `values`: the largest (darkest) 12-bit reading of each 5 ms window, from
  the test image's `[bc] trace` dumps. A dump starts 100 ms before the
  first window above 500 and ends after 400 ms below 500; the time between
  dumps is filled with the last white value (or 230). So anything that
  never rose above 500 outside a dump is missing, and bars read a little
  wide (the largest value of each window).
- `dist_mm`: `[ms from the first window, distance_state dist_mm]` from the
  `[motion] distance_state` console lines, about every 200 ms (whole mm,
  modulo 32768).
- `log`, `from`, `to`: where they came from.

They were made by a small script that read those dumps and console lines
(not in the repo; the logs are not in the repo either).
