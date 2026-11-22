# Lab run 22 Nov 2026, 15:57 to 16:03 (image lab_straddle65w)

Two clean runs on branch `buddy2/straddle-lab-22nov` (source of the image,
see `docs/motion/LAB_STRADDLE65W.md`):

- 15:58 box bypass, back on the line 0 mm / 0 deg; S-curves (short Line
  Searches found the line in about 0.4 s).
- 16:00 barcode Z, left turn at the junction (86 deg on the IMU), box
  bypass back on the line 2 mm / 0 deg; hump 8.6 mm high, 246 mm long.

Files:
- `wifi_console_1557-1603.log`: the car's console over Wi-Fi (netlog).
- `usb_console_1555-1603.log`: the USB console around the flash.
- `decoder_1557-1603.log`: the viewer's telemetry decoder.
- `build_manifest_line.tsv`: the build line (flags, sha256) of the image.
