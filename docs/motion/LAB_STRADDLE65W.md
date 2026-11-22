# Lab image lab_straddle65w (22 Nov 2026)

Buddy 2 (motion) lab branch. Test code with many build flags, **not for
master as is**. It is the source of the image that ran the course cleanly
on 22 Nov, 15:57 to 16:03: box bypass back on the line 0 mm / 0 deg and
2 mm / 0 deg, barcode Z then a left turn at the junction (86 deg), both
S-curves, hump 8.6 mm.

## Hardware it needs

- The two line sensors **straddle** the line: the gap between the two
  heads is a little wider than the 25 mm line (about 3 cm), so both read
  white when the car is centred and one reads black when the car drifts.
- Left encoder Grove cable (GP0/GP1) firmly in: a loose one shows as
  "left wheel stalled" at the start.

## What changed (all behind build flags, defaults unchanged)

- `MOTION_LF_STRADDLE`: Line Following for the straddle layout, after the
  other team's `b3_line_step` (used with permission): self-tuned nudge on a
  touch, curve memory, inner-wheel pivot in hairpins.
- Lost line: the line sits between the sensors, so it can only leave past
  one. A sensor losing the line while the car turns away from it, or the
  line sweeping across both, means it went out that side: pivot back on
  the inner wheel. After a recovery or a bypass, no touch within 150 mm
  means Line Search. 1.5 m with no touch at all is the last resort.
- `LINE_JUNCTION_PAIR`: after a barcode, the next time both line sensors
  are black is the junction (no distance window). Turn: forward 135 mm,
  pivot on the IMU (`MOTION_TURN_IMU`) and stop when the new line is
  between the sensors.
- Bypass: the line's direction is taken from the box (it sits on the line,
  `MOTION_SM_BOX_DIR`), a line crossing nearly side-on (both sensors black
  at once within 40 deg of the line's direction) is not taken as the line
  (`MOTION_SM_CROSS_CHECK_MM`), no barcode read and no junction through the
  bypass and 200 mm after (`LINE_SCAN_AFTER_BYPASS_MM`), Line Search if no
  line 80 mm past where it should be (`MOTION_SM_MISS_X_MM`).
- Steering stays live over a barcode: the barcode sensor also reads a
  curve's line as a barcode.

## Build (Docker, as the tested image)

Wi-Fi and broker settings come from your own private
`config/wifi_credentials.h` and `config/broker_config.h` (never committed).
The tested image used the barcode table `LFFFFLFFFFFFFFFFLFFFFFFFFL`
(A, F, Q and Z turn left, the rest straight): set it on the day, or add
`-DBARCODE_DEFAULT_TABLE_SET=1` and the table to the flags.

```sh
cd build_make
make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
    WIFI_DNS=1 WIFI_MQTT=1 WIFI_NETLOG=1 GCC='arm-none-eabi-gcc \
    -DBARCODE_SENSOR_ANALOG=1 \
    -DBC_PART_GAMBLE=1 \
    -DBC_PART_CELLS_MAX=40000 \
    -DHUMP_TRACK_MM=120 \
    -DLINE_BARCODE_HOLD=1 \
    -DLINE_BARCODE_HOLD_STEER=0 \
    -DLINE_SCAN_MAX_TURN_DPS=15 \
    -DLINE_LOST_MS=60000u \
    -DLINE_CRACK_MS=0u \
    -DLINE_JUNCTION_PAIR=1 \
    -DLINE_PAIR_SAMPLES=20u \
    -DLINE_PAIR_MM10=175u \
    -DLINE_PAIR_GP_WINDOW=60000u \
    -DMOTION_TURN_FWD_MM=135 \
    -DMOTION_TURN_IMU=1 \
    -DMOTION_LF_STRADDLE=1 \
    -DMOTION_ST_LOST_MM=1500u \
    -DMOTION_ST_BASE_MMPS=65 \
    -DMOTION_ST_CURVE_MMPS=40 \
    -DMOTION_ST_MIN_WHEEL_MMPS=20 \
    -DMOTION_ST_KICK_CURV=5500 \
    -DMOTION_ST_LEARN_CURV=450 \
    -DMOTION_ST_LEARN_TIGHT_CURV=600 \
    -DMOTION_ST_SLEW_CURV=600 \
    -DMOTION_SM_BOX_DIR=1 \
    -DMOTION_SM_CROSS_CHECK_MM=8 \
    -DMOTION_SM_MISS_X_MM=80 \
    -DMOTION_SM_SEEN_DEG=12 \
    -DMOTION_SM_ONTO_MM=117 \
    -DMOTION_LF_HOLD_ON_BARCODE=0 \
    -DMOTION_LF_BASE=6 \
    -DMOTION_LF_BASE_CURVE=4 \
    -DMOTION_LF_CURVE_BOOST_X10=15 \
    -DMOTION_LF_BASE_READ=3 \
    -DMOTION_LF_GAIN=5 \
    -DMOTION_LF_DAMP=4 \
    -DMOTION_LF_SLEW=25 \
    -DMOTION_LF_RATE_DAMP=12 \
    -DMOTION_LF_EDGE_LOST=1 \
    -DMOTION_LF_EDGE_MIN_TICKS=15u \
    -DMOTION_LF_REALIGN=0 \
    -DMOTION_LF_GAP_MM=60 \
    -DMOTION_LF_LOST_DIR_MS=300 \
    -DMOTION_LS_DIR_MIN=25 \
    -DMOTION_LS_SWEEP_DEG=80 \
    -DMOTION_BC_RETRY=1 \
    -DIR_ADAPT_BLACK_UP_SHIFT=8 \
    -DIR_ADAPT_BLACK_DOWN_SHIFT=13 \
    -DBC_DELTA=160 \
    -DBC_DELTA_MIN=120 \
    -DLINE_JUNCTION_SAMPLES=450 \
    -DLINE_ARMED_HOLD_MM=1000000 \
    -DSL_KNOWN_W_MM=90 \
    -DSL_WIDTH_EDGE_MID=1 \
    -DSL_EDGE_REFINE=1 \
    -DSL_BEAM_HALF_DEG=10 \
    -DMOTION_BYPASS_SMOOTH=1 \
    -DMOTION_SM_ADAPT=0 \
    -DMOTION_SM_HD_FALLBACK=1 \
    -DMOTION_HD_LINE_MM=120 \
    -DMOTION_HD_SNAP_DEG=20 \
    -DMOTION_SM_Y_MM=215 \
    -DMOTION_SM_SHIFT_MIN=0 \
    -DMOTION_SM_S_OUT_MM=210 \
    -DMOTION_SM_S_IN_MM=510 \
    -DMOTION_SM_L_IN_MM=100 \
    -DMOTION_SM_MAX_DEG=60 \
    -DMOTION_SM_SPEED=13 \
    -DMOTION_SM_SEEN_X_MM=250 \
    -DMOTION_SM_WATCH=2 \
    -DMOTION_SM_WATCH_DEG=55 \
    -DMOTION_SM_CLEAR_MM=250 \
    -DSERVO_LIMIT_DEG=55 \
    -DSCAN_WATCH_DEG=55'
```

`WIFI_NETLOG=1` is the lab console over UDP; leave it out for demo day.
