# Week 12 check-in: what Buddy 4 can show

imu_task (tilt, hump detection and height, abnormal-motion safety stop,
`app_program/imu_task/`). Follows [`week10_checkin.md`](week10_checkin.md);
the full evidence is in [`terrain_analysis_report.md`](terrain_analysis_report.md)
(§ numbers below refer to it). Numbers come from the **self-driven** runs on
the integrated car, 1–3 Nov 2026, on branch `imu_task` (commits `1551f7d`,
`6859033` and after).

**The demo ramp** is about the height of the smaller practice ramp (the prof,
3 Nov); the higher ramp is out of the demo.

## 1. Against the requirements

| Requirement | Limit | What we have | Status |
|---|---|---|---|
| NFR-05 hump height | ±2 mm or 20 % (limit ±5 mm) | Demo hump (10 mm), self-driven: **14/14 within target**, mean 9.96 mm (−0.04), range 8.9–11.3 mm (§4.5). 23 mm, no longer required: target 3/4, limit 4/4 | ✅ |
| FR-10 count each hump once | — | 18 self-driven crossings; two humps in one mission twice (10 + 23 mm; two 10 mm humps ~19 cm apart); obstacle stop on the hump, reversed off it, no second hump; 3 flat runs, 0 humps | ✅ |
| FR-12 highest hump per mission | — | `humps: 2 (highest 28 mm)`; in `E imu` and `S.hgt` | ✅ |
| FR-23 IMU events in telemetry | — | **`E imu`** on every class change and hump-count change: class, count, height. `S.hum` / `S.hgt` now filled from it (read 0 before) | ✅ (new) |
| FR-29 tilt and turn rate, 1 s | — | `S.tlt` from the 100 ms mean (0 on flat, 4–7 on the hump); `S.trn` left +, right − | ✅ |
| FR-27 motion classes | — | moving, turning, climbing, descending, stopped match the car; impact A raised 1.5 → **2.5 g** (normal driving reached 2.0 g) | ✅ (impact not driven into a real obstacle) |
| FR-28 abnormal tilt | T ≥ steepest + 5° | T = 25°; demo ramp peaks ~7°; 23 mm ramp 20°; tip-over faults 4/4 | ✅ |
| **Picked up in a mission** | the prof's request | in hand 1.2 s in Running / Paused → IMU fault; not in Ready. On the car: fault 1.1 s after `picked up` | ✅ (new) |
| NFR-19 tilt → STOP_IMU | 40 ms | **20.5–21.2 ms**, 10 trials, 4 on the integrated car | ✅ |
| NFR-18 STOP_IMU → brake | 20 ms | 47–61 µs | ✅ |
| NFR-13 IMU fault | no data 50 ms | cal fail, tilt, read failure, pick-up; recovery by GP20 / `cmd reset`, then a new mission drives | ✅ (init failure not run) |
| NFR-11 sample period | 10 ms ± 1 | bench, Wi-Fi off: all within. Car build: a few windows up to ~1.7 ms late (shared with motion) | ❌ known limitation; no effect on height (encoder distance) or the tilt fault |

## 2. Changed since Week 10

| Change | Why (what the car showed) |
|---|---|
| Hump closes after **120 mm** level (was 250) | the demo hump stayed open ~25 cm past the crest; now ~14–15 cm |
| A stop or end of mission **counts a hump past its top** | a stop ~8 cm after the hump lost an 11 mm hump; the line may end right after the hump |
| Lean sign fixed; lines say which side is up | left read as RIGHT; checked still on a book |
| No false `picked up` after braking (500 ms settle) | every stop printed one |
| Slow bends count as turns, angle from 3 °/s | the right bend counted as no turn ("1 left, 0 right"); now 1 right, ~4° under |
| `S.tlt` from the 100 ms mean | single samples read 8–18° on flat floor |
| `E imu` record (M6) | the only task not sending E records; `S.hum` / `S.hgt` read 0 |
| IMU fault when picked up in a mission | the prof |
| Impact A 1.5 → 2.5 g | normal driving reached 1.5–2.0 g |

## 3. Repeatable on the spot

- **Hump:** drive over the demo hump: `===== HUMP 1: 9.x mm high =====`, then
  `E imu hump_count=1 hump_height_mm=10` and `S.hum=1 hgt=10` in the viewer.
- **Tilt fault:** tip the car past 25° in a mission: `STOPPING: car tipped
  over (... deg to the LEFT, right side up)`, caught in ~21 ms.
- **Pick-up fault:** pause, lift and hold: `STOPPING: car picked up (in hand
  1.2 s)`.
- **Recovery:** GP20 (or `cmd reset`) → `cal ok` → a new mission drives.
- **PC tests** (no board): `docker compose run --rm build bash -lc "make -C
  tests"` → `imu_hump` 29/29 scenarios, `imu_safety` 98 checks.

## 4. Known limitations (§6)

1. NFR-11 jitter in the car build (above).
2. The pick-up fault needs the wheels still: lifted while driving, it comes
   ~2 s after the car loses the line and stops (unless the lift tilts it
   past 25° first).
3. A hump crossed at an angle reads high (+4.8 mm on the bend vs +2.6 square
   on, 23 mm). The demo hump is on a straight.
4. Turn angles from the encoders: count right, angle ~4° under.
5. 5987 mg boot glitch: 0 of 4 logged boots, cause not found. Check `cal ok`
   after power-on; reboot if it shows 5987.
6. No gyroscope; magnetometer works but is unused.

## 5. Demo day

- Flash the **car build** from master (`... WIFI_DNS=1 WIFI_MQTT=1`), with
  `TM_WIFI_REPORT 0`; never the `WIFI_NETLOG` image.
- Car across the line, flat and still, at power-on (IMU and line calibration).
- Check `[imu] cal ok` (no 5987).
- Lift it onto the start in Ready, not during a mission.
