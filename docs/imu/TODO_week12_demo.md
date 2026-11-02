# imu_task: to do before the Week 12 final demo (Buddy 4)

What is left after the Week 10 check-in (`week10_checkin.md`), in the order
to do it. Tick items off as they are done; write the numbers in the blanks.

## 0. Switch to my own console output (first, before anything else)

From here on, work on `imu_task`, not master: it has master's features
(120 mm hump close, `FAULT_IMU`, `imu_ts`) plus the readable console
(`d84590b`).

- [ ] `git switch imu_task`, Docker build (`SMP=1 CONSOLE=usb_cdc`), flash
      `build_make/mtk3pico_smp1_usb_cdc.uf2`
- [ ] Watch the console on the bench: status line, `>>>` / `<<<` event
      lines, hump progress, mission summary; note anything unclear to fix
- [ ] **Axis signs:** tilt the car left by hand, console says LEFT; nose up,
      console says nose UP. If not, flip `IMU_LEFT_SIGN` / `IMU_NOSE_SIGN`
      in `imu_events.h`, rebuild, check again
- [ ] Push the car round by hand: a left turn prints as left, a right turn
      as right, counts go up
- [ ] Use this build for every run in section 2
- [ ] After the PR to master: update `week10_checkin.md` §6 (or a Week 12
      version) with the new console lines

## 1. Code (do first, so the runs test the final firmware)

- [ ] **`E imu` record (M6):** send IMU events to comms through
      `comm_w_pack_imu()`; check it in the viewer as `E imu`
- [ ] Fix anything unclear found in section 0's console check
- [ ] **PR `imu_task` → master:** readable console (`d84590b`), merge fix
      (`df9ee24`), check-in doc, the `E imu` record; build passes, host tests pass
- [ ] **5987 mg boot glitch:** count it across boots (____ of ____), find
      the cause, or reboot-and-recalibrate as the fallback on demo day

## 2. On the car, self-driving

### S6: prof's hump (195 × 10 mm)

Crossings 1-2 done on 1 Nov: 9.7 mm, 9.6 mm.

- [ ] Crossing 3: height ____ mm, closes ____ cm after the crest, no fault, no STOP_SCAN, line kept
- [ ] Crossing 4: height ____ mm, closes ____ cm after the crest, no fault, no STOP_SCAN, line kept
- [ ] Crossing 5: height ____ mm, closes ____ cm after the crest, no fault, no STOP_SCAN, line kept
- [ ] Hump closes ~13 cm after the crest (the `HUMP_TOP_MAX_MM` 120 fix, `698d48b`)
- [ ] Save the viewer logs for all 5 crossings

### NFR-05: height accuracy (`IMU_CSV 1` build)

Target ±2 mm or 20 %, limit ±5 mm.

- [ ] 10 mm ramp, 5 passes: ____ / ____ / ____ / ____ / ____ mm
- [ ] 23.5 mm ramp, 5 passes: ____ / ____ / ____ / ____ / ____ mm
- [ ] Note any wheel skid (one wheel's count freezes)

### No false humps

- [ ] Full lap on flat floor, hump count stays at 0

### FR-27: motion classes

- [ ] `[imu] motion:` lines match what the car does: turning, still, climbing, descending
- [ ] Normal driving never shows **impact** (1.5 g)
- [ ] Retune the thresholds from the CSV logs if the classes are wrong

### Turns (turn rate from the encoders, no gyro)

- [ ] Curves on the line: class goes to `turning` (> 20 °/s) and back (< 12 °/s), no flicker on straights
- [ ] `S.trn` sign in the viewer: a left turn reads positive (CCW)
- [ ] Turn angle against the real one: 90° turn reads ____°, 180° reads ____° (branch console)
- [ ] Left / right turn counts and heading at mission end match the course (branch console)
- [ ] Wiggles while line following are not counted (`EV_TURN_MIN_DEG` 15°); retune `EV_TURN_*` if they are
- [ ] Wheel skid on the hump does not count as a turn
- [ ] When junction turns run (S5, Buddy 2, after GP7 is replaced): each turn counted once, right direction

### NFR-11: timing (10 ms ± 1)

- [ ] Driving, Wi-Fi off: sample period ____..____ µs, every window within
- [ ] Driving, Wi-Fi on (car build): sample period ____..____ µs
- [ ] Share the results with Buddy 2 (motion shows the same jitter)

### FR-28 / NFR-19: tilt fault (limit 40 ms)

- [ ] Tilt past 25° during a mission: it brakes, fault time ____ ms
- [ ] Recalibrate on GP20, then check the car can start a new mission (fault recovery)

## 3. With the team (S8 full mission)

- [ ] Full-course runs with the hump in place: hump counted once, height
      in S / `E imu`, no false IMU fault (____ of 5 runs)
- [ ] Highest hump reported at the end of the mission (FR-12)

## 4. Docs

- [ ] Add the self-driven results to `terrain_analysis_report.md` §4 and §7
- [ ] Update `week10_checkin.md` (or a Week 12 version) with the new numbers
- [ ] Ask the integration owner to update the S6 row and M6 in `docs/INTEGRATION.md`

## 5. Demo day

- [ ] Flash the **car build** from master (`... WIFI_DNS=1 WIFI_MQTT=1`),
      with `TM_WIFI_REPORT 0`; never the `WIFI_NETLOG` image
- [ ] Calibrate on **flat floor**, not on the ramp
- [ ] IMU wiring (GP16 SDA, GP17 SCL) firm; check for the 5987 mg glitch
      after boot, reboot if seen
- [ ] Live points ready: hump height on the prof's hump, tilt fault and
      its time (under 40 ms), recovery by recalibration (GP20)
- [ ] Evidence ready: host tests (`make -C tests`: `imu_hump`, `imu_safety`),
      saved logs, the requirements table
