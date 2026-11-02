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
- [x] **Axis signs:** tilt the car left by hand, console says LEFT; nose up,
      console says nose UP. If not, flip `IMU_LEFT_SIGN` / `IMU_NOSE_SIGN`
      in `imu_events.h`, rebuild, check again. *(2 Nov: left/right was
      swapped, `IMU_LEFT_SIGN` now -1; nose up correct as +1)*
- [ ] Push the car round by hand: a left turn prints as left, a right turn
      as right, counts go up
- [ ] Use this build for every run in section 2
- [ ] After the PR to master: update `week10_checkin.md` §6 (or a Week 12
      version) with the new console lines

## 1. Code (do first, so the runs test the final firmware)

- [x] **`E imu` record (M6):** send IMU events to comms through
      `comm_w_pack_imu()`; check it in the viewer as `E imu`
      *(3 Nov: `send_imu_record()` in `imu_task.c`, on every class change
      and hump-count change. On the car: class 6 at the start, class 2 on
      the climb, `hump_count=1 hump_height_mm=10` at the hump, class 0 at
      the stop, count 0 at mission start; `S.hum=1 S.hgt=10`, which read 0
      before)*
- [ ] Fix anything unclear found in section 0's console check
- [ ] **PR `imu_task` → master:** readable console (`d84590b`), merge fix
      (`df9ee24`), check-in doc, the `E imu` record; build passes, host tests pass
- [x] **5987 mg boot glitch:** count it across boots (**0 of 4** logged
      boots, 2–3 Nov: 23:41, 02:01, 02:57, 02:59; plus ~8 unlogged reboots
      after flashing), find the cause, or reboot-and-recalibrate as the
      fallback on demo day. *Cause not found; not seen since Week 10. Kept
      as the demo-day check (§5)*

## 2. On the car, self-driving

### S6: prof's hump (195 × 10 mm)

Crossings 1-2 done on 1 Nov: 9.7 mm, 9.6 mm.

- [ ] Crossing 3 (2 Nov): height **9.5 mm** (up 9.5, down 0.0, steepest
      7°, 234 mm long), closes **~14 cm** after the crest *(from times at
      ~48 mm/s)*, no IMU fault during the crossing; STOP_SCAN / line kept
      still to confirm. A second nose-up 10-13° ~10 cm later was the USB
      cable dragging (not the hump); run the rest cable-free (`WIFI_NETLOG`)
- *Attempt (2 Nov, Wi-Fi):* peaked at 11 mm, but GP21 ~80 mm after the
  hump dropped it ("hump discarded (mission paused)") before the 120 mm
  level close. Fixed: a pause or mission end now counts a hump past its
  top (`hump_end_early()` in `hump.c`, 4 new host scenarios, 29/29).
  Rerun with the fix.
- [ ] Crossing 4 (3 Nov, Wi-Fi, no cable): height **9.9 mm** (up 9.9,
      down 0.0, steepest 6°, 244 mm long), closes **~15 cm** after the crest
      (closed by the 120 mm level rule before any stop), no IMU fault;
      STOP_SCAN / line kept still to confirm. After it: two right pivots
      (26°, 18°) with stops, counted as turns: line search? to check
- [ ] Crossing 5 (3 Nov, Wi-Fi, no cable): height **9.2 mm** (up 9.2,
      down 0.0, steepest 6°, 239 mm long), closes **~14 cm** after the
      crest, no IMU fault; STOP_SCAN / line kept still to confirm
- [x] Hump closes ~13 cm after the crest (the `HUMP_TOP_MAX_MM` 120 fix,
      `698d48b`): **~14, ~15, ~14 cm** on crossings 3-5
- **Two humps, one run (3 Nov, half track):** HUMP 1 **10.5 mm** (244 mm
  long, 7°), HUMP 2 **27.8 mm** (292 mm long, 15°, on the right bend, just
  before the line ends); `humps: 2 (highest 28 mm)` (FR-10, FR-12). HUMP 2
  closed by the end-of-line pause: "mission paused past its top, closed
  now", the pause fix's first run on the car. Real sizes: 10 / **23 mm**,
  so HUMP 2 read **+4.8 mm (+21 %)**: just outside the 20 % target (4.6 mm),
  inside the ±5 mm limit. Climbed on the bend with "nose DOWN 8, leaning
  LEFT 12": likely at an angle, and tilt magnitude × distance over-reads a
  slope crossed at an angle
- **23 mm hump on the straight (3 Nov):** **25.6 mm** (+2.6 mm, +11 %:
  within target), 287 mm long, 16°. Better than on the bend (+4.8). But the
  climb again read "nose DOWN 8, leaning LEFT 11" with the car square on,
  as every climb tonight did. Book test (still, hands off): front wheels on
  a book read "nose UP 15", left wheels "leaning RIGHT 10 (left side up)",
  each on one axis only: **the labels and mounting are right**. So the nose
  DOWN + lean LEFT while climbing is how the car really rides onto these
  humps (right wheel first?), not a label bug. Height and safety use the
  tilt magnitude and are not affected
- **Hump + obstacle + bend, one run (3 Nov, slow-bend and S.tlt fixes):**
  hand put in front while the car was on the hump: stopped there, HUMP 1
  **10.1 mm** counted ("stopped on top"), REVERSE ~150 mm back down without
  a second hump ("on level ground again"), "false alarm", drove on. Bend
  (heading 8° → 57° right) counted as **1 right turn** (36°: the first
  ~13° under the bend threshold), summary 0 left / 1 right: correct. No
  false "picked up" after 3 stops. S.tlt 0 on flat, ≤ 5 on the hump
- **Angle fix (`EV_BEND_PRE_DPS` 3, 3 Nov):** bend heading ~1° → 45° right
  read **40°** (was 36 for 49, 39 for 47 before), 1 right turn; HUMP 1
  **10.7 mm**; MQTT `cmd start` / `cmd pause` from the viewer worked
- **S6 heights, all 5:** 9.7 / 9.6 / 9.5 / 9.9 / 9.2 mm on the 10 mm hump
  (mean 9.6, worst -0.8 mm): all within the ±2 mm target
- [ ] Save the viewer logs for all 5 crossings

**Prof's email (3 Nov): the demo ramp is about the height of the smaller
practice ramp, slightly lower or higher; the higher ramp can be
disregarded** (clearance and scraping). So the 23 mm passes below are extra
evidence, not required. 23 mm self-driven, 3 Nov: 27.8 (on the bend), 25.6,
26.2, 26.0 mm: target 3/4, limit 4/4. A third run (02:38) caught on the
ramp: slewed 100° left, reared to 20°, tipped 32° nose up → tilt fault
(correct). Steepest seen on it: 20°, only 5° under T.

### NFR-05: height accuracy (`IMU_CSV 1` build)

Target ±2 mm or 20 %, limit ±5 mm.

- [ ] 10 mm ramp, 5 passes: ____ / ____ / ____ / ____ / ____ mm
- [ ] 23.5 mm ramp, 5 passes: ____ / ____ / ____ / ____ / ____ mm
- [ ] Note any wheel skid (one wheel's count freezes)

### No false humps

- [ ] Full lap on flat floor, hump count stays at 0 *(3 Nov: ~1.4 m, 29 s
      at 48 mm/s up to the right bend, hump count 0, no lean / nose / bump
      events, classes only moving and turning. Not a full lap: only half
      the track was laid, and the line ends just after the right bend, so
      the car swept in line search (to 170° and back) and stopped there, as
      on crossings 4 and 5. Repeat on the full track)*

### FR-27: motion classes

- [ ] `[imu] motion:` lines match what the car does: turning, still, climbing, descending
- [x] Normal driving never shows **impact** (1.5 g) *(3 Nov: it did, 1.5–2.0
      g on the hump's edge, the bend, braking out of a pivot. A raised to
      **2.5 g** (`MC_IMPACT_MG`); host test: 1.5–2.0 g knocks are not
      impacts. To confirm on one run: no `BUMP` in normal driving)*
- [ ] Retune the thresholds from the CSV logs if the classes are wrong

### Turns (turn rate from the encoders, no gyro)

- [ ] Curves on the line: class goes to `turning` (> 20 °/s) and back (< 12 °/s), no flicker on straights
- [ ] `S.trn` sign in the viewer: a left turn reads positive (CCW)
- [ ] Turn angle against the real one: 90° turn reads ____°, 180° reads ____° (branch console)
      *(3 Nov, the right bend after the hump: heading ends at 99-100°
      right, correct, but the turn reads only 51° / 26° + 18°: the slow
      start of the bend, ~6°/s, is under `EV_TURN_PRE_DPS` 12 and is not
      summed. Line-search sweeps (right to 151°, then back left) merge into
      the same turn)*
- [ ] Left / right turn counts and heading at mission end match the course (branch console)
- [ ] Wiggles while line following are not counted (`EV_TURN_MIN_DEG` 15°); retune `EV_TURN_*` if they are
- [ ] Wheel skid on the hump does not count as a turn
- [ ] When junction turns run (S5, Buddy 2, after GP7 is replaced): each turn counted once, right direction

### NFR-11: timing (10 ms ± 1)

- [ ] Driving, Wi-Fi off: sample period ____..____ µs, every window within
- [ ] Driving, Wi-Fi on (car build): sample period ____..____ µs
      *(3 Nov, stopped, Wi-Fi on, `WIFI_NETLOG` + `IMU_VERBOSE` build, the
      heaviest console load: **6 of 16** 10 s windows outside ±1 ms, worst
      8 214..11 814 µs, work max 4 796 µs; late/early samples in pairs, so
      task wake-up latency, not missed samples)*
- [x] **Decided (3 Nov): NFR-11 is a known limitation, no rerun.** The car
      build already missed it while driving in S4 (17/19, worst 11.7 ms), so
      a plain-build repeat would only confirm it; the timing line also needs
      `IMU_VERBOSE`, which adds load. No effect on the results: height uses
      encoder distance, not time; the tilt fault stays at ~21 ms of 40
- **Magnetometer works** (3 Nov, `IMU_VERBOSE`): turned flat in 4 × 90°,
  x/y swing ±400 mG, steps 87/89/89/95° from atan2, z steady 245-270 mG,
  |B| ≈ 455 mG, hard-iron offset ≈ (-66, +29) mG, noise ±3 mG. Unused by
  the firmware (known limitation)
- [ ] Share the results with Buddy 2 (motion shows the same jitter)

### Picked up in a mission → IMU fault (the prof, 3 Nov)

- [x] Built: in hand 1.2 s (`EV_LIFT_FAULT_MS`) in Running or Paused →
      `STOPPING: car picked up` + STOP_IMU + FAULT; not in Ready.
      *(3 Nov 02:34:16: paused, lifted → fault 1.1 s after `picked up`;
      lifted while driving → tipped 26° on the way up → tilt fault 21.2 ms;
      `cmd reset` → cal ok → new mission drove)*
- [ ] No false lift fault in normal runs (obstacle stops, scans, end of line)

### FR-28 / NFR-19: tilt fault (limit 40 ms)

- [x] Tilt past 25° during a mission: it brakes, fault time **20.9 ms**
      (2 Nov, tipped by hand to 26.5° left, `imu_task` branch build)
- [x] Recalibrate on GP20, then check the car can start a new mission (fault
      recovery): cal ok, FAULT cleared, new mission drove at ~48 mm/s

## 3. With the team (S8 full mission)

- [ ] Full-course runs with the hump in place: hump counted once, height
      in S / `E imu`, no false IMU fault (____ of 5 runs)
- [ ] Highest hump reported at the end of the mission (FR-12)

## 4. Docs

- [x] Add the self-driven results to `terrain_analysis_report.md` §4 and §7
      (3 Nov: new §3.6 and §4.5, §5 / §5.1 / §6 / §7 updated)
- [x] Update `week10_checkin.md` (or a Week 12 version) with the new numbers
      *(3 Nov: `week12_checkin.md`)*
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
