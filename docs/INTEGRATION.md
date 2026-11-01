# Integration baseline: the integrated car (1 Nov 2026)

This is the **starting baseline** for the integrated firmware on `master`. It
records what integration built, changed and measured, the deviations from the
submitted design (Week 6 report, Tribunal 1), and what is still open.

**Where this document wins.** Where any other document in `docs/` disagrees with
this one about the integrated car, this one is current. The others stay as
subsystem detail and history: `docs/comms/` (Wi-Fi and telemetry), `docs/motion/`,
`docs/imu/`, `docs/obstacle/`, `BUDDY3_LINE_BARCODE.md`, `ARCHITECTURE.md`,
`PORT_RP2040.md`, `GETTING_STARTED.md`. Where code and docs disagree, the code is
the truth.

**How it was done.** Buddy 1 integrated on the `integration` branch (off `buddy_1`
at `e980b93`, 29 Oct), with the team's permission to change any subsystem. Each
change to someone else's subsystem is its own commit, marked `(for Buddy N)`, so
its owner can find it and explain it. Stages S0-S8 were tested on the car in
order. The evidence logs (viewer captures, per stage and attempt) are kept by
Buddy 1 and available on request.

---

## 1. Build and flash

Always through Docker (`AGENTS.md` section 1). The images in use:

| Image | Command (`cd build_make && make … -j8`) | Use |
|---|---|---|
| **Car build** | `SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1` → `mtk3pico_smp1_usb_cdc_wifi_mqtt.uf2` | Every stage test; the basis of the demo build |
| **Wireless console** (debug only) | the car build `+ WIFI_NETLOG=1` → `…_wifi_mqtt_netlog.uf2` | The whole console over UDP to the viewer: debugging with no cable (section 9) |

- **Stage images use `CONSOLE=usb_cdc`.** A UART build prints to GP0/GP1, not the
  cable.
- **Config:** `config/wifi_credentials.h` and `config/broker_config.h` (gitignored;
  copy the `.example.h`). Two optional settings in `broker_config.h`:
  - `UTK_MQTT_BROKER_DISCOVER 1` makes the car scan its /24 for the broker port
    (bench only, section 3.4).
  - `UTK_NETLOG_ADDRESS` / `UTK_NETLOG_PORT` say where the wireless console goes.
- **Demo day:** the car build, with `TM_WIFI_REPORT 0` (`app_main.c`; it turns the
  `[wifi]`/`[mqtt]` console task off), `UTK_MQTT_BROKER_DISCOVER 0`, the given
  broker address in `broker_config.h`, and **never** `WIFI_NETLOG`.
- **Development switches,** all off in the car build:
  - `LINE_COMMISSIONING_MODE` (`common/robot_config.h`): line samples at 10/s for
    the viewer's line chart.
  - `LINE_GP7_DIAG` (`line_task/barcode_scanner.h`): GP7 wiring check.
  - `NAV_TEST_JUNCTION_ACTION` (`nav_task/nav_ctrl.c`, default -1): the turn to
    take at a junction with no barcode.
  - `OBSTACLE_TEST_MODE` (`obstacle_task.c`): 0 = ultrasonic range test,
    4 = normal.
  - `TM_WORD_STRESS`, `TM_RACE_TEST`, `TM_BLINK_LED`.

## 2. What runs where

Lower number = higher priority. Core 0 = `TP_PRC1`, core 1 = `TP_PRC2`. Every task
is pinned (`AGENTS.md` section 7).

| Task | Owner | Priority | Core | Job |
|---|---|---|---|---|
| `cyw43_utk_task` | Buddy 1 (lib) | 4 | 0 | Radio and lwIP owner (NO_SYS): Wi-Fi, DHCP/DNS, MQTT, wireless console |
| `ir_sense_task` | Buddy 3 | 10 | 1 | 1 ms line + GP7 sampling, line_state word, junctions |
| `motion_task` | Buddy 2 | 11 | 1 | The only motor writer: PID, statechart, every manoeuvre, stall/encoder faults |
| `imu_task` | Buddy 4 | 12 | 1 | Tilt, humps, abnormal motion → STOP_IMU |
| `scan_task` | Buddy 5 | 13 | 1 | Ultrasonic + servo: STOP_SCAN, scans, planning, bypass looks |
| `barcode_task` | Buddy 3 | 14 | 0 | Barcode decode, action table, map from MQTT |
| `nav_ctrl_task` | Buddy 5 (Vehicle Controller) | 15 | 0 | Mission state, buttons GP20/GP21, buzzer, calibration, junction commands |
| `comms_task` | Buddy 1 | 16 | 0 | S/H/E/D records, inbound commands |
| `wifi_status_task` | Buddy 1 | 17 | 0 | `[wifi]`/`[mqtt]` console lines (`TM_WIFI_REPORT`) |
| `motion_report` | Buddy 2 | 24 | 0 | Prints motion's reports off core 1 |

Kernel-object limits raised for the integrated car in S0 (`config/config.h`):
mutexes 8, message buffers 12, cyclic handlers 8, alarms 8.

## 3. Interfaces built during integration

### 3.1 Command buffers into `motion_task` (M1)

| Buffer | Sender → receiver | Room | Content |
|---|---|---|---|
| `nav_cmd_mbf` | nav → motion | 4 | JUNCTION (action), FAULT_CLEAR(IMU), FAULT_CLEAR(MOTION); 2 raw bytes `{NAV_CMD_*, arg}` (D4) |
| `avoid_cmd_mbf` | scan → motion | 2 | One packed 16-bit word (`common/messages.h`): REVERSE, BYPASS (side + lateral offset), **LOOK** (op 3, added in S7: the reading of a bypass probe) |

Both are created by `motor_task_start()` and bound in `app_main.c`
(`nav_bind()`, `scan_if_bind()`). FAULT_CLEAR(IMU) is sent only after both
calibrations pass, and motion then clears STOP_IMU, so an IMU fault recovers
through Calibrating.

### 3.2 scan_flg bits (`obstacle_task/scan_if.h`)

| Bit | Set by | Meaning |
|---|---|---|
| `REVERSE_DONE` 0x04, `LINE_FOUND` 0x08, `LINE_LOST` 0x10 | motion | As designed |
| `LOOK` 0x20 | motion | Bypass probe: car pivoted 45° toward the obstacle, braked; look 45° further (square-on) and answer with an AVOID LOOK |
| `LOOK_AHEAD` 0x40 | motion | Look straight along the sidestep lane |
| `SIDE_BLOCKED` 0x80 | motion | The lane was blocked; the car is back on its heading; re-plan the other side |

### 3.3 Shared words

- **Read by motion:** `line_state` (Buddy 3) and the IMU's `hump_state` word
  (Buddy 4, `imu_word_read()`).
- **Read by scan:** `hump_state`, and motion's `distance_state` (one aligned load,
  for the pivot-aware re-plan, D13).
- **Read by many tasks:** the mission word (nav).

All are lock-free single-writer words (D1-D3).

### 3.4 Telemetry and comms

- **Records:** E records from motion (every statechart change), mission (every
  state change), line, barcode and obstacle. S.mis is read from nav every second.
- **Barcode map:** a checked map from MQTT goes to `barcode_task` (M8b).
- **SCANNING:** a new MQTT session state (5) for broker discovery. It shows on the
  console and is reported as "connecting" in D.
- **Retry back-off:** 2 s, 4 s, then 5 s (was up to 15 s; D7).
- **Broker discovery** (`UTK_MQTT_BROKER_DISCOVER`, off by default): tries the
  configured address, then the last broker that accepted the car, then hosts
  .1-.254 of the car's /24, 8 probes at a time.

## 4. Stage results

| Stage | Result | Evidence *(measured on the car unless marked)* |
|---|---|---|
| S0 baseline | **Pass** | Everything boots together; kernel limits raised; first line state at boot |
| S1 mission + comms | **Pass** | E mission records on every change; S.mis live |
| S2 safety chain | **Pass** (FR-30 partial) | NFR-18 stop ≤ 314 µs (stand-in) / 217 µs (real senders); NFR-19 21.4 ms; FR-30 brakes and faults. Lock-free word stress test (Tribunal C row 2): 18/18 pass |
| S3 line sensing (by hand) | **Pass**, except barcode decode | Line lost/found, junctions, map → barcode_task (M8b). Barcode decode **open**: the KY-033 cannot resolve 2 mm bars |
| S4 line following | **Pass** | Base 4: 1 m in 32 s = 3.1 cm/s; 5/5 laps, 0 LOST. **Now base 6: 4 laps, 0 LOST, ~31 s a lap** (47.5 s at base 4), so ~4.7 cm/s *(calculated from the lap times)*. NFR-02 proposed "3 cm/s, 4 of 5 runs". NFR-11: motion 36/39 and imu 17/19 windows within 10 ms ±1 (max 11.5 / 11.7 ms), **open** |
| S5 junction turns | **Built, not run on the car** | Sim only. **Blocked by hardware:** the GP7 KY-033 reads stuck HIGH, and must sit beside the line (D11) |
| S6 hump | **Partly tested**: 2 of the planned 5 crossings; the hump-close fix not run on the car | Prof's hump (195 mm, 10 mm): 9.7 / 9.6 mm (NFR-05 OK); no IMU fault, no false STOP_SCAN, line not lost. The V_hump cap and 2 × t_lost never fired (speeds under the cap, no LOST). Hump mode should now end ~13 cm after the crest, not ~25 *(host test)* |
| S7 obstacle | **Works**; NFR-07 not met | Probe bypass passes boxes of unknown depth and rejoins the line (user, attempts 3-4). Side choice by clearance; reverse when < 100 mm. NFR-07's 7 s is out of reach: the scan alone takes 3.9-5.3 s, the bypass ~20-30 s. Sim: 200 mm box 26.4 s, 500 mm box 34.5 s |
| S8 full mission | **Not started** | D7 done. Missing: end-of-course trigger (D10), ≥ 5 full-course runs, AP/broker outage runs (NFR-09) |

## 5. Changes per subsystem

Commit hashes are on `integration`; see section 11 for the full list.

### Buddy 2: motion (`app_program/motor_task/`)

- **The two command buffers and motion fault bits** (`5ac5c1b`); E motion record
  on every statechart change (`754fdf7`).
- **Line following** (`e3e69c5`, `5044314`, `4afe976`):
  - soft PD steering with a damper;
  - off the line, turn toward the side last seen at full error (fixed the
    curve-apex run-offs);
  - **base 6** with gain and damper scaled to the base (7 / 22).
  - Base 8 swung left and right: the slew limit, hard-turn timer and damper
    window are fixed in time, not distance.
- **Turning** (`3580c38`): drive `MOTION_TURN_FWD_MM` (50, PROPOSED: GP7 to axle,
  to measure), pivot 90° / 180° by encoder count (`MOTION_TURN_SPEED` 6), resume
  after a stop. The same commit held the first bypass (fixed 45° legs and
  L_pass), replaced by the probe bypass below.
- **Bypass**, the probe bypass (`44b2955`, `bf892b5`, D14). By encoder count, for
  obstacles of unknown size:
  1. **Sidestep:** pivot 70° out and look along the lane (servo 0°). Anything
     closer than leg + 205 mm (capped at 511): pivot back to the heading and set
     SIDE_BLOCKED, so scan re-plans the other side from its original profile.
     No new command within 5 s: give up into Line Search. Otherwise sidestep
     offset / sin 70° and pivot back.
  2. **Alongside:** 250 then 100 mm steps. After each, pivot 45° toward the box,
     scan looks square-on, pivot back. A reading below 450 mm, no echo, or no
     answer within 3 s counts as "still there".
  3. **Passed** = clear (≥ 450 mm) after the box was seen once, or clear and
     never seen after 450 mm alongside (shallower than a step); then 120 mm
     more. Still beside it at 1.5 m: no turn in, so Line Search ends in
     LINE_LOST (FR-18 "otherwise stop").
  4. **Back in:** pivot 70° in, drive until GP26/GP27 see the line (at most
     leg + 150 mm), 125 mm more (sensor to axle), pivot onto the heading.
  - Bypass pivots 10 counts/tick, legs 12. Pivots are scaled to 90 % (90°
    turned ~100° at that speed). A stop on the way brakes; if it clears with
    no new command the bypass carries on; a new BYPASS starts over.
- **Hump** (`a0f9eba`, M3): every wheel target capped at V_hump 15 cm/s
  (18 counts/tick) while `hump_state` ≠ NONE. On a hump the line may be missing
  2 × t_lost (3 s) before Line Search.

### Buddy 3: line and barcode (`app_program/line_task/`)

- Merged `origin/line_task` (`4c0da60`, `fa36c5b`); `TM_LINE_TASK 1`; first line
  state at boot (`7107d5a`).
- **Adaptive per-sensor thresholds** (`ff35bbb`): each sensor keeps its own white
  and black levels and follows them while driving. 900/700 are only the boot
  guess.
- **Calibration passes with one sensor on the line edge** (`67a7e09`).
- `LINE_GP7_DIAG` wiring check (`df96cb6`).

### Buddy 4: IMU (`app_program/imu_task/`)

- The demo hump closes after 120 mm of level ground, not 250 (`698d48b`). On the
  prof's hump the car never reads the way down, because its body is about as long
  as the hump. Host scenarios added for that hump.

### Buddy 5: scan and nav (`app_program/obstacle_task/`, `app_program/nav_task/`)

- **nav:**
  - FAULT_CLEAR arguments (`7bc75fd`);
  - E mission record (`6afeac3`);
  - `NAV_TEST_JUNCTION_ACTION` (`057157a`).
- **scan:**
  - `avoid_cmd_mbf` bound (`98f7dd2`).
  - **Measured geometry** (`ce80cad`, D12): car 125 mm wide, axle to front
    155 mm, to back 70 mm. C_pass 218 mm (pivot sweep 168 + 50). The path
    corridor stays half width + margin (113). Max offset 400 mm.
  - **Pivot-aware re-plan** (D13): while bypassing, re-plan below D_stop on
    straight legs, below D_rev while still or pivoting.
  - **Bypass looks and side switch** (D14).

### Buddy 1: comms and tools

- **From `buddy_1`, not yet on `master`:**
  - Wi-Fi-loss fault bit 5, latched into S, and the stall restart (S refused
    twice while CONNECTED → reconnect) (`d3d697b`; docs `cf33ff6`, `88e2498`);
  - subscribing to `cmd` and `map`, the inbound mailbox and the retained D
    state (`4ff2389`);
  - `snap`, `ping`, mission commands and the map check (M8a) (`25bbce2`; docs
    `e980b93`);
  - viewer: sends commands, bad-input tests, broker stop/start (`5d810be`);
  - the PR #17 build workaround (`0a71379`), reverted once the missing files
    arrived (`1b6abae`);
  - a merge of `origin/master` into `buddy_1` (`c22694e`).
- **Comms:**
  - S.mis from nav (`e77c62c`);
  - checked map → barcode_task (`14adcf7`);
  - broker discovery (`d324bca`);
  - back-off cap 5 s (`e2c4603`);
  - wireless console (`4bdc9b2`).
- **Viewer** (`tools/telemetry_viewer.py`):
  - remembers the filter and names it in saved logs (`41762da`, `66dcee5`);
  - live GP26/GP27 line chart (`2f4dd8f`);
  - wireless console source (`4dc6f4d`).
- **Tests:** the lock-free word stress test (`555f3a5`).

## 6. Deviations from the submitted design

**D** = differs from Week 6 / Tribunal 1. **M** = in the design, was missing.

| # | What | Status |
|---|---|---|
| D1-D3 | `hump_state`, `line_state` and the mission state are lock-free single-writer words, not mutex-protected | **Kept (decided 29 Oct).** One aligned 32-bit store and load; stress test pass; disassembly checked |
| D4 | nav commands are 2 raw bytes, not the packed 16-bit layout | Open (low risk) |
| D5 | barcode → nav through a pending-command word, not `bar_cmd_mbf` | Open |
| D6 | inbound commands via a message buffer, not a mailbox | Kept |
| D7 | MQTT back-off was 2 → 15 s; the design says fixed 2-5 s | **Fixed:** capped at 5 s |
| D8 | comms paced by timeouts, not a 1 s cyclic handler | Kept (H spacing 0.989-1.012 s) |
| D9 | MQTT mission commands (start/pause/resume/reset/finish) | Kept |
| D10 | **No end-of-course trigger**: Finished only by command | **Open: team decision** |
| D11 | Line sensors both inside the line; junction from GP7 only | Kept; **GP7 must sit beside the line** |
| D12 | C_pass = pivot sweep + margin (218 mm), not half width + margin | Decided 1 Nov |
| D13 | While bypassing, re-plan below D_rev (not D_stop) while still or pivoting | Decided 1 Nov |
| D14 | Probe bypass instead of fixed 45° legs + L_pass; new LOOK op and scan_flg bits 0x20/0x40/0x80; lane check; 70° legs | Decided 1 Nov (robustness first: the demo obstacle size is not known) |
| M1 | `nav_cmd_mbf`, `avoid_cmd_mbf` | Done |
| M2 | Line Following, Turning, Bypass, Line Search | Done (Turning not run on the car) |
| M3 | V_hump cap | Done |
| M4 | Battery sense on GP28 | **Not built** |
| M5 | RGB LED (GP18) | **Not built** |
| M6 | E records from every task | motion, mission, line, barcode, obstacle done; **imu not sending E** |
| M7 | Rejected-junction telemetry | Not verified |
| M8 | Left motor coasts at boot until motion starts | Not measured |

## 7. Tuned values (and where they came from)

| Value | Setting | Source |
|---|---|---|
| Line following base | 6 counts/tick (~4.7 cm/s), gain 7, damper 22 | S6 attempt 1: 4 laps clean; base 8 oscillated |
| Line thresholds | adaptive per sensor; boot guess 900/700 | S4 attempt 6 |
| `LINE_LOST_MS` | 1500 (× 2 on a hump) | design t_lost |
| Turn forward | `MOTION_TURN_FWD_MM` 50 | **PROPOSED**, measure GP7 → axle |
| Car geometry | 125 mm wide, axle → front 155, → back 70, line sensors → axle 125 | measured 1 Nov |
| C_pass / corridor / max offset | 218 / 113 / 400 mm | D12 |
| Bypass | 70° legs, first step 250, steps 100, seen < 450 mm, run-out 120, max 1.5 m, pivots × 0.9 | S7 attempts 1-4 |
| HC-SR04 at 45° incidence | box invisible (~3.7 m reading) | echo test 1 Nov: why probes are square-on |
| Hump close | `HUMP_TOP_MAX_MM` 120 | S6, prof's hump |
| V_hump | 150 mm/s = 18 counts/tick | design |
| MQTT back-off | 2 / 4 / 5 s | D7 |

## 8. Known issues and open items

1. **GP7 sensor dead** (stuck HIGH; KY-033 inferred faulty). Junctions and
   barcodes cannot be tested until it is replaced and mounted beside the line.
2. **End-of-course trigger** (D10): needs a team decision.
3. **NFR-07** (bypass ≤ 7 s) cannot be met at these speeds; report it as a
   measured shortfall.
4. **NFR-11** tick jitter while driving (worst 11.5 ms); cause not found.
5. **Barcode decode:** the KY-033 cannot resolve 2 mm bars even once GP7 works.
6. **IMU:** x = y = z = 5987 mg at some boots (Buddy 4); no E imu record (M6).
7. **Not built:** battery sense (M4), RGB LED (M5).
8. **Not yet run on the car:**
   - the hump close at 120 mm;
   - the 5 s MQTT back-off (NFR-09 to re-measure);
   - the wireless console;
   - the turning code (S5).
9. **Tests:** `tests/scan_logic_test.c` and `tests/host/test_messages.c` are not in
   `tests/Makefile`. Run the first by hand (see its header). The second does not
   cover the LOOK op.

## 9. Tools: the viewer

`py tools\telemetry_viewer.py` (or the exe): one page with the MQTT records
(decoded), the serial console and the broker log.

- **Saving:** saves `serial.log`, `decoder.log`, `broker.log`; a saved car-log
  names its filter.
- **Line chart** of GP26/GP27 (with `LINE_COMMISSIONING_MODE 1`).
- **Wireless console** (`WIFI_NETLOG=1` images, debug only):
  - The car sends its whole console over UDP to port 31884. The viewer listens
    there (`--netlog`, 0 = off) and shows the lines as `serial`.
  - Lost datagrams and bytes the car dropped show as `viewer` lines.
  - Allow UDP 31884 in the PC firewall, and set the serial port to none on
    wireless runs.
  - On the car: a 16 KB ring in `tm_com.c` (copied in `tm_snd_dat()`, never
    waits; drops are counted) is drained by the radio task
    (`lwip_utk_netlog.c`) in datagrams of up to 1 KB, whole lines. In this
    profile `tm_usb_state()` reports the console ready at once, because the
    tasks' 20 s wait for a USB host would stall every boot on battery.
  - Flashing still needs the cable, and the last lines before a crash or
    watchdog reset can be lost.
- **Over MQTT alone** (no cable, no netlog) you only get the S/E/H/D records.
  Fault reasons, bypass phases, raw line values and tick timing are console
  only.

## 10. Tests

Host suites: `docker compose run --rm build bash -lc "make -C tests"`. They
cover:

- **motion:** lib, statechart, motor, watchdog, encoder, the motion task sim
  (bypass with probes, lane-blocked switch, give-up cases, hump cap,
  2 × t_lost);
- **IMU:** hump, including the prof's hump, 25 scenarios; safety;
- **line:** calibration, tracking, line state, barcode.

The scan logic test runs by hand:
`gcc -std=c99 -Wall -Wextra -I app_program/obstacle_task tests/scan_logic_test.c app_program/obstacle_task/scan_logic.c`.

## 11. Commits on `integration` since `buddy_1` (`e980b93`)

```
555f3a5 test(race_test): lock-free shared-word stress test (Tribunal 1 C row 2)
4c0da60 week 6 code for barcode and line reader
fa36c5b Merge origin/line_task into integration (for Buddy 3)
1b6abae Revert "fix(build): comment out references to files missing from PR #17" (for Buddy 3)
a7d1bec test(motion_task_sim): stub line_state_word() for line following (for Buddy 2)
e4fb1b3 config: raise kernel-object limits for the integrated car (S0)
6a5b4dc chore(app): delete app_main.c.old
7107d5a fix(line): send the first line state at boot (for Buddy 3)
df96cb6 test(line): LINE_GP7_DIAG wiring check for the barcode sensor (for Buddy 3)
41762da feat(tools): viewer remembers the filter, saved logs name it
7bc75fd refactor(nav): FAULT_CLEAR arguments in nav_ctrl.h (for Buddy 5)
5ac5c1b feat(motion): nav_cmd_mbf and avoid_cmd_mbf, motion fault bits (for Buddy 2)
1c1cc8a feat(app): bind nav_cmd_mbf to nav_ctrl_task
6afeac3 feat(nav): E mission record on every state change (for Buddy 5)
e77c62c feat(comms): S.mis read from nav every second
66dcee5 feat(tools): a filter overrides the hidden tags
14adcf7 feat(comms): M8b, a checked map goes to barcode_task
1eb0737 docs(comms): M8b built and tested on the car (S3)
754fdf7 feat(motion): E motion record on every statechart change (for Buddy 2)
e3e69c5 feat(motion): slower, softer line following with a damper (for Buddy 2)
2f4dd8f feat(tools): live GP26/GP27 line sensor chart in the viewer
ff35bbb feat(line): adaptive per-sensor line thresholds (for Buddy 3)
5044314 fix(motion): keep turning to the last-seen side off the line (for Buddy 2)
67a7e09 fix(line): pass calibration with one sensor on the line edge (for Buddy 3)
057157a feat(nav): test action for junctions without a barcode (for Buddy 5)
3580c38 feat(motion): turning at junctions and the obstacle bypass (for Buddy 2)
98f7dd2 feat(obstacle): bind avoid_cmd_mbf to scan_task (for Buddy 5)
ce80cad feat(obstacle): geometry, pivot-aware re-plan, bypass looks (for Buddy 5)
44b2955 feat(motion): probe bypass for obstacles of unknown size (for Buddy 2)
d324bca feat(comms): optional MQTT broker discovery on the local /24
bf892b5 fix(motion): bypass probe "seen" below 450 mm, 120 mm run-out (for Buddy 2)
a0f9eba feat(motion): V_hump cap and 2 x t_lost on a hump (for Buddy 2)
4afe976 feat(motion): line following at base 6, steering scaled to speed (for Buddy 2)
698d48b fix(imu): close the demo hump after 120 mm level, not 250 (for Buddy 4)
e2c4603 fix(comms): cap the MQTT retry back-off at 5 s (D7)
326a141 docs(comms): MQTT back-off now capped at 5 s
4bdc9b2 feat(comms): wireless console debug profile (WIFI_NETLOG=1)
4dc6f4d feat(tools): viewer shows the wireless console (WIFI_NETLOG=1 images)
e2fccdb docs: WIFI_NETLOG debug profile in AGENTS.md and the example config
071657b docs: integration baseline (docs/INTEGRATION.md)
```

(plus the review fixes to this file after `071657b`.)

Against `origin/master` (`340da59`, PR #17), `integration` adds the 40 commits
above (and the review commit), plus 9 from `buddy_1` that had not reached
`master`:
- the Wi-Fi-loss fault bit and the stall restart;
- inbound `cmd` / `map` with the inbound mailbox and retained state (M8a);
- viewer commands;
- their docs.

`origin/master` is an ancestor of `integration`, so the push is a fast-forward.
Buddy 1's Wi-Fi and telemetry work is documented in `docs/comms/README.md`.
