# Terrain analysis report — imu_task (Buddy 4)

Team IS-24 · Pico Racer · µT-Kernel 3.0 on RP2040 (SMP) · tests run 2026-10-27/28
Code: `app_program/imu_task/` (`imu_task.c`, `hump.c`, `imu_safety.c`, `imu_class.c`) on branch `imu_task`, commit `981208e`.
Data representation (Week 13): [`week13_representation.md`](week13_representation.md).

This report covers the hump detection and height method (FR-10, FR-11, FR-12), its
accuracy against NFR-05, the thresholds chosen from measured data, and the IMU fault
behaviour (FR-02, FR-28, NFR-13, NFR-18/19). Every number below comes from the
serial logs of the runs listed in §5; nothing is presented as validated that was not
measured.

---

## 1. Setup

| Item | Value |
|---|---|
| Sensor | GY-511 board, identified at boot as **LSM303D** (ID 0x49 at 0x1D) — not the LSM303DLHC named in earlier docs |
| Bus | I2C0, SDA GP16, SCL GP17 |
| Range / resolution | ±8 g, 0.244 mg/LSB (bumps do not clip; at ±2 g they pinned at 1998 mg) |
| Sample rate | 10 ms cyclic handler (100 Hz output data rate); magnetometer read but not used |
| Distance | encoder `dist_mm` from Buddy 2's `distance_state` word (wrap-safe 15-bit deltas), read on core 1 |
| Task | imu_task, priority 12, pinned to core 1 (`TA_ASSPRC`) |
| Console | one `[imu]` status line per second (FR-29 rate), events printed as they happen |

**LSM303D, not LSM303DLHC.** Our GY-511 boards carry the LSM303D variant: accelerometer
and magnetometer share one I2C address, 0x1D (the DLHC uses 0x19 and 0x1E). The code
identifies the chip by its WHO_AM_I register instead of assuming one, and uses the
LSM303D register map, checked against the datasheet:

| Register / value | LSM303D datasheet | Code (`imu_task.c`) |
|---|---|---|
| Address | 0x1D (SA0 high), 0x1E (SA0 low) | tries 0x1D, then 0x1E |
| WHO_AM_I | 0x0F = 0x49 | checked at boot |
| Accel output | OUT_X_L_A 0x28 … OUT_Z_H_A 0x2D, little-endian | 6 bytes from 0x28, auto-increment (0x80) |
| Mag output | OUT_X_L_M 0x08 … OUT_Z_H_M 0x0D | 6 bytes from 0x08, auto-increment |
| CTRL1 0x20 | 100 Hz, BDU, X/Y/Z enabled | 0x6F |
| CTRL2 0x21 | ±8 g | 0x18 |
| CTRL5 0x24 | mag high resolution, 6.25 Hz | 0x64 |
| CTRL6 0x25 | ±4 gauss | 0x20 |
| CTRL7 0x26 | continuous conversion | 0x00 |
| Accel scale | 0.244 mg/LSB at ±8 g | × 244 / 1000 |
| Mag scale | 0.160 mgauss/LSB at ±4 gauss | × 4 / 25 |

Still and level, the car reads 0, 0, 1000 mg (±2 mg), which confirms the scale.

**How the runs were done.** The normal firmware keeps the motors braked, so every
run was a **hand push** over the ramp with both wheels on the ground. Encoders still
count while pushed. This matters for the error analysis (§6): with the motors braked,
a wheel with less grip skids instead of rolling.

## 2. Calibration and noise

At boot, and on every CAL_REQ from nav_ctrl_task, the car must be still for 1 s:
spread on every axis ≤ 80 mg, wheels not turning, and mean within ±150 mg of
(0, 0, 1000 mg). The mean becomes the zero offset. After 5 failed tries it answers
CAL_FAIL (FR-02).

| Measured | Value |
|---|---|
| Offsets found (across boots) | x 8…78 mg, y −11…−48 mg, z 55…70 mg |
| Spread during a passing calibration | 16…78 mg |
| Noise, still, 1 s means | x, y within ±2 mg; z 990…1008 mg |
| Level difference between floor spots | up to ~30 mg (~1.7°) |

The floor/ramp-top difference is why hump onset is set above 30 mg (§3.2).

## 3. Method

### 3.1 Height from tilt × distance

Hump detection works on **100 ms windows** (10 samples). For each window:

- tilt vector = mean (x, y) minus the *level baseline* (the calibration level,
  updated only while stopped near level);
- tilt magnitude `t` in mg = 1000·sin θ, so it works whatever way the board is
  mounted;
- distance driven `Δs` in mm from `distance_state`;
- rise `Δh = t · Δs` in µm (1000·sin θ · mm = µm), signed: nose-up and forward, or
  nose-down and reversing, is up.

Height is the running sum, kept in µm (sub-mm fixed point; at 15 cm/s and 5° one
window adds ~1.3 mm, resolved to 1 µm). The reported height is the peak of the sum.
**No correction factor is applied** (§4.5).

### 3.2 Hump onset, end and special cases

| Rule | Value | Why |
|---|---|---|
| Onset | tilt ≥ 50 mg (~2.9°) for 3 windows (300 ms), moving, motion STEADY | a single jolting window once set "uphill" backwards and lost a 23 mm hump |
| Uphill direction | sum of the 3 onset windows | mounting angle does not matter |
| Level | tilt < 35 mg (~2.0°) | floor spots differ by up to 30 mg |
| End, driven over | back within 2 mm of start height, or level for 40 mm **once on the way down** | a level top longer than 40 mm otherwise ended the hump on the top and counted the way down as a second hump |
| End, safety cap | level for 250 mm | far side that drops off too fast to read as a slope |
| End, stopped | still for 1 s: hump counted, then the car is followed "on top" until it is back down | backing off a ramp once counted as a new hump |
| Stopped on the slope | still, tilt ≥ 65 mg and under the 4 mm minimum: the climb stays open for the next push | stopped 2.4 mm up the 10 mm ramp, the climb was ignored and the rest of the ramp was never counted (2026-10-28) |
| Picked up | stopped but tilt changes > 60 mg between windows: "on top" ends | carried back to the start, the next climb was swallowed |
| Minimum | 4 mm | shorter climbs are reported as ignored |
| Fault / tilt past T | unfinished hump discarded (Statechart 6.4) | held on its side, a few mm of wheel turn once read as a 6 mm "hump" |
| Accelerating / decelerating | windows that are not STEADY do not start a hump or flip CLIMBING/DESCENDING | acceleration reads as tilt (pitch hold) |

`hump_state` (NONE / CLIMBING / DESCENDING) follows the direction of travel.

### 3.3 Mission handling (FR-12, Statechart 6.4)

Ready → Running starts a new mission: the hump count and the **highest hump** go
back to zero. Paused → Running keeps them. Leaving Running (pause, end, fault)
drops an unfinished hump. A pause keeps the "on top" follow of a hump already
counted, since the car has not moved: paused on top and then backed down, the way
down once counted as a second hump. Verified on the car: `[imu] mission start:
humps and highest hump reset` on every GP20 start (2026-10-28 02:28–02:32), and
`hump discarded (mission paused)` for a climb paused part way up.

### 3.4 Motion event classification (FR-27)

`imu_class.c` classifies every 10 ms sample into a 3-bit code; a change is
printed (`[imu] motion: …`) once it has held 100 ms, an impact at once.

| Code | Class | Rule (thresholds proposed, see §6) |
|---|---|---|
| 0 | still | no encoder mm for 100 ms and \|turn\| < 5 °/s |
| 1 | turning | \|turn\| > 20 °/s, until < 12 °/s (hysteresis) |
| 2 / 3 | climbing / descending | from the hump detector |
| 4 | impact | \|a\| > 1.5 g on one sample, held 500 ms; only within 1 s of the wheels turning |
| 5 | accelerating | `distance_state` says ACCELERATING |
| 6 | moving | straight, steady speed |

Choices from the car logs: "still" needs 100 ms, not the proposed 50 ms, because
`dist_mm` counts whole mm (at 30 mm/s a count comes only every 33 ms); turning has
hysteresis because a turn rate wandering 17–21 °/s flipped moving/turning every
~50 ms; impact is gated on motion because carrying the car by hand reached 2.4 g.
Turn rate comes from the encoders, so a skidding wheel also reads as turning.

### 3.5 Shared state: the `hump_state` word

Everything imu_task reports to other tasks is one aligned 32-bit word, written
whole every 10 ms by imu_task alone, read with one load and no mutex (Tribunal
Version C): hump state 2 bits, class 3, height now 8, count 4, highest hump 8,
fault 1, tilt 6. Layout and sizes in the Week 13 table.

All of these cases are replayed on a PC: `tests/imu_hump` (23 scenarios, each one a
pass that went wrong on the car) and `tests/imu_safety` (61 checks). Both pass.

## 4. Results

NFR-05 (revised 2026-10-28): each estimate within **±2 mm or 20 %, whichever is
larger** (target), and never more than **±5 mm** (limit), over 5 traversals per
reference hump with the car **driving itself**, both wheels rolling. Traversals
with wheel skid are reported, but separately, as a known limitation (§6).

Why revised: the first proposal, 0.5 cm or 20 %, is ±5 mm for every hump under
25 mm, so on a 9–10 mm hump half the true height would still pass. The clean
passes below hold about ±2 mm, which is the accuracy the method shows; ±2 mm is
also far finer than the gap between the reference humps (10 mm and 23.5 mm), which
is what FR-12 (report the highest hump) needs. The ±5 mm limit is kept as the pass
mark for runs that are not clean.

All runs below are **hand pushes** (§1), so none of them is yet the self-driven
evidence NFR-05 asks for; they are scored against both bounds to show where the
method stands.

### 4.1 Lower practice ramp — 10 mm (ruler); target ±2.0 mm, limit ±5 mm

Passes pushed fully over the ramp:

| Pass | Reported (mm) | Error (mm) | Peak tilt | Wheels | Target | Limit |
|---|---|---|---|---|---|---|
| 1 | 9.0 | −1.0 | 116 mg (6.7°) | both rolling | ✅ | ✅ |
| 2 | 8.9 | −1.1 | 127 mg | both rolling | ✅ | ✅ |
| 3 | 6.7 | −3.3 | 173 mg | left wheel skidding | ❌ | ✅ |
| 4 | 6.3 | −3.7 | 164 mg | left wheel skidding | ❌ | ✅ |
| 5 | 6.9 | −3.1 | 166 mg | left wheel stalled | ❌ | ✅ |
| 6 | 9.4 | −0.6 | 112 mg | both rolling | ✅ | ✅ |
| 7 | 7.9 | −2.1 | 119 mg | left wheel froze near top | ❌ | ✅ |
| 8 | 8.8 | −1.2 | 126 mg | slight slew | ✅ | ✅ |

**Target 4/8, limit 8/8.** Mean 8.0 mm (−2.0 mm). Clean passes (1, 2, 6):
−0.6…−1.1 mm, all within target; every target miss had a stalled or skidding wheel.

### 4.2 Higher practice ramp — 23.5 mm (ruler 23–24 mm); target ±4.7 mm (20 %), limit ±5 mm

| Pass | Reported (mm) | Error (mm) | Peak tilt | Notes | Target | Limit |
|---|---|---|---|---|---|---|
| 1 | 23.0 | −0.5 | 264 mg (15.3°) | clean | ✅ | ✅ |
| 2 | 25.3 | +1.8 | 259 mg | clean | ✅ | ✅ |
| 3 | 15.9 | −7.6 | 271 mg (15.7°) | left wheel froze | ❌ | ❌ |
| 4 | 13.0 | −10.5 | 216 mg | fast, jerky push; hump ended early | ❌ | ❌ |
| 5 | 28.2 | +4.7 | 246 mg | fastest push (~120 mm/s) | ✅ | ✅ |

**3/5 on both; 3/3 clean passes pass.** Mean of clean passes 25.5 mm (+2.0 mm,
+8.5 %). The plan says to tune on the lower ramp and ignore the higher one; this
run is kept as evidence that the method still works on a hump more than twice as
tall.

### 4.3 Shimmed ramp — 12 mm (lower ramp + 2 mm shim); target ±2.4 mm (20 %), limit ±5 mm

| Pass | Reported (mm) | Error (mm) | Peak tilt | Notes | Target | Limit |
|---|---|---|---|---|---|---|
| 1 | 13.7 | +1.7 | 261 mg | clean | ✅ | ✅ |
| 2 | 8.4 | −3.6 | 157 mg | left wheel froze (reads low) | ❌ | ✅ |
| 3 | 17.7 | +5.7 | 133 mg | jerky push: right wheel spun to 2350 c/s while the left nearly stopped (reads high) | ❌ | ❌ |

**Target 1/3, limit 2/3.** Only 3 traversals counted: two more on this setup were
discarded as abnormal tilt under the old T = 20° (§4.6), so this setup still needs
2 more clean passes for the 5 that NFR-05 asks for. Pass 3 is the first over-read:
skid goes either way, depending on which wheel slips.

### 4.4 Lower practice ramp, final firmware (2026-10-28) — 10 mm; target ±2.0 mm, limit ±5 mm

The same 10 mm ramp as §4.1. Car calibrated on the flat floor, mission RUNNING,
commit `981208e`. Each pass pushed up the ramp and stopped on top.

| Pass | Reported (mm) | Error (mm) | Peak tilt | Wheels | Target | Limit |
|---|---|---|---|---|---|---|
| 1 | 6.6 | −3.4 | 114 mg | pivoting: right wheel up to ~4× left, up to 31 °/s | ❌ | ✅ |
| 2 | 6.5 | −3.5 | 199 mg | pivoting: left wheel nearly stopped, up to 117 °/s | ❌ | ✅ |
| 3 | 6.6 | −3.4 | 140 mg | pivoting: left wheel stopped, up to 94 °/s | ❌ | ✅ |

**Target 0/3, limit 3/3**, and exactly one hump per pass with no false fault. The
three agree within 0.1 mm, so the under-read is systematic, and every pass
pivoted: distance is the mean of both wheels, which under-reads a pivot (§6). The
straightest push of the day, on an earlier build of the same session, read
7.6 mm (−2.4 mm). The pivoting passes read 2.3–2.9 mm lower than the clean
passes of §4.1 on the same ramp (−0.6…−1.1 mm). Self-driven passes are needed to tell the method's own error
from the push.

### 4.5 Why no correction factor

Clean passes read about **10 % low** on the 10 mm ramp and about **8 % high** on the
23.5 mm ramp. No single multiplier fits both, and the large errors come from wheel
skid and pivoting (§6), which a factor cannot fix: a factor fitted to the skidding
passes would push clean passes over the limit. The method is used uncorrected;
every clean traversal recorded is within the NFR-05 target.

### 4.6 Thresholds: proposed → chosen from data

| Threshold | Proposed (Week 6) | Final | Evidence |
|---|---|---|---|
| T (abnormal tilt) | 20° | **25°** (423 mg) | steepest hump driven peaked at 15.7°; on it, two jerky pushes held > 20° for 3 samples, so 20° would have been a false STOP in a mission. FR-28 needs T ≥ steepest + 5°. |
| N | 3 samples | 3 (30 ms) | unchanged |
| Jolt filter | — | samples with \|a\| outside 0.7…1.3 g do not count to N | crashes push x/y but pull \|a\| away from 1 g |
| Knock filter | — | a sample over T only counts if z < 950 mg as well (cos 25° = 906 mg) | a sideways knock under ~0.8 g stays inside the \|a\| band and read 25.7–26.9° on a level car: two false faults on 2026-10-28 (IMPACT while paused; pushed onto the ramp edge), status under 4° straight after |
| Slope hold | — | 65 mg | stopped on the 10 mm ramp at 4.6–5.9° (80–103 mg); a car left on a hump edge after a pivot rests at ~50 mg |
| Y (no data) | 50 ms | 50 ms (5 samples) | unchanged |
| θ_on | 5° for 3 samples | **2.9° (50 mg) for 3 × 100 ms windows** | a 10 mm ramp peaks at only 6–10°, so a lower onset catches more of the climb; windows (not single samples) keep it above the 1.7° floor variation |
| θ_off | 3° | **2.0° (35 mg)**, plus 40 mm on the way down | floor spots differ by up to 1.7° |
| A (impact) | 1.5 g | not implemented yet | carrying the car reached 2.4 g, so impact must be gated on motion |

Peak tilt of the reference humps: 10 mm ramp 5.6…10°, 23.5 mm ramp 12.5…15.7°. With
T = 25° the lower (demo) ramp has a 15° margin.

**Verified on the car with T = 25°** (2026-10-28 01:47): pushes over the 23.5 mm ramp
in RUNNING, jerky ones included, gave no fault; a hand tilt in RUNNING faulted at
26.3°.

## 5. Fault behaviour (FR-02, FR-28, NFR-13, NFR-18)

| Test | Log (time) | Result |
|---|---|---|
| Boot, still: CAL_REQ → CAL_OK_IMU → READY | 00:14:28 | ✅ |
| Car moving at boot: 5 tries → CAL_FAIL → nav FAULT | 00:17:27 | ✅ |
| Recalibrate from FAULT with GP20 → READY | 00:16:22, 00:17:07, 00:18:08 | ✅ |
| Tilt in RUNNING (T = 20° build) → IMU_FAULT + STOP_IMU | first fault run (log without timestamps), "34.9 deg"; brake **58 µs** after request | ✅ |
| Tilt in RUNNING (T = 25° build) | 01:49:00 "26.3 deg"; brake **48 µs** | ✅ |
| SDA pulled in READY → no-data fault | first bad read → fault in **37 ms** (00:15:05.888 → .925); READY → FAULT | ✅ (Y = 50 ms) |
| SDA pulled in RUNNING → fault + STOP_IMU | 01:22:22.106 → brake 01:22:22.141 (**47 µs** after request, ~45 ms after first bad read) | ✅ |
| Chip re-init after failed reads; data returns; FAULT stays until recalibration | 00:15:45 | ✅ (Statechart 6.4) |
| No false hump while handling the car | 00:18:33 (tilted 40–44° in READY) | ✅ |
| Knock in PAUSED, before the knock filter | 2026-10-28 log, t = 47.8 s: `IMPACT`, "26.7 deg" fault, status 0.2° next line | ❌ false fault → fixed (§4.6) |
| Knock / tap with the knock filter | 2026-10-28 logs: `IMPACT` 6 times (READY, RUNNING, PAUSED), no tilt fault | ✅ |
| Hand tilt with the knock filter | 2026-10-28: RUNNING "25.3 deg" and PAUSED "27.2 deg", **20.7 ms** each; brake 43–45 µs | ✅ |
| Knock that also dips z | 2026-10-28 log, t = 82 s: PAUSED, `IMPACT`, "25.7 deg" fault | ❌ one false fault left (§6) |
| Boot with IMU unplugged → CAL_FAIL | — | not run |

Brake time is motion_task's own stamp (`motion_stop_stamp`), so NFR-18 is met by a
wide margin.

### 5.1 Timing (NFR-11, NFR-19)

Measured on the 1 MHz timer by imu_task itself (Wi-Fi off; the Wi-Fi-on repeat
needs Buddy 1's MQTT build):

| Requirement | Measured | Limit | Result |
|---|---|---|---|
| NFR-11 sample period | 9 653–10 352 µs over 22 windows of 10 s | 10 ms ± 1 ms | ✅ |
| NFR-11 work per sample (WCET) | worst 1 466 µs since boot, typically 950–1 250 µs | — (15 % of 10 ms) | ✅ |
| NFR-19 first sample over T → STOP_IMU | 20.8, 20.5, 20.5, 20.5 ms (4 trials, 2026-10-28 02:28–02:32); 20.7, 20.7 ms with the knock filter | 40 ms | ✅ |

NFR-19 is fixed by the design: the fault needs N = 3 samples in a row, which arrive
at 0, 10 and 20 ms, and it is raised ~0.5 ms into the third; the four trials agree
to 0.3 ms. 4 trials were run instead of the 100 planned, as the spread shows no
variation worth sampling further.

## 6. Limitations and error sources

1. **Wheel skid is the main error.** In every session the left wheel stalled on the
   climb on some passes (`count L` frozen while R rose, fake turn rate 20–110 °/s).
   Distance is the mean of both wheels, so the height reads low when a wheel
   stalls, and high when one spins (12 mm pass 3). Pushed by hand with
   braked motors this is likely worse than under motor power; the runs must be
   repeated with the car driving itself before the method is called final. Reported
   to Buddy 2, along with `missed R` counts rising to 20.
2. **No gyroscope.** Tilt is from the accelerometer only, so acceleration reads as
   tilt. Non-STEADY windows are gated (§3.2), and jolts are excluded from the fault
   check, but hard pushes still disturb the estimate (23.5 mm pass 4).
3. **Pausing on the slope drops the climb.** A climb stopped part way up stays
   open (§3.2), but a mission pause discards an unfinished hump, as planned. Not
   expected on the course: the car drives up and over without pausing.
4. **Pivoting under-reads.** Distance is the mean of both wheels, so a push that
   swings onto the ramp on one wheel reads low: −3.4 mm on 10 mm, three times
   (§4.4).
5. **A knock that also dips z can still trip T.** The knock filter needs z to
   drop as a real tilt does; one knock in PAUSED did both and faulted at 25.7°.
   The `IMU_CSV` build can capture the raw samples to tune it.
6. **Calibration accepts a tilted car.** x and y may be up to ±150 mg from level,
   so calibrating on the ramp (5.7°) was accepted, and every tilt after it read
   ~6° off (one 15 mm "hump" on the 10 mm ramp). Calibrate on flat floor.
7. **Half-connected SDA.** With the wire loose, half the reads failed and the rest
   read 30–65° while the car was (probably) level. The outcome is still a fault, but
   wrong-yet-valid readings are possible.
8. **STOP_IMU stays set until reboot.** nav sends FAULT_CLEAR(IMU) after a passing
   calibration, but `nav_cmd_mbf` is not bound to motion_task yet (Buddy 2, step 1.7).
9. **All runs so far are hand pushes.** NFR-05, the classifier thresholds (turn,
   still, impact A = 1.5 g) and flat-floor runs (no false humps) are to be repeated
   with the car driving itself; the CSV logging build (`IMU_CSV`) is ready for it.
10. **Not implemented yet:** sending IMU events to comms_task (`comm_w_pack_imu` /
   `comms_tele_send`, plan 2.3, with Buddy 1; tilt and turn rate are not in the
   telemetry schema yet), NFR-11 timing with Wi-Fi on.
11. **Magnetometer** is read but unused.

## 7. Requirements status

| ID | Status |
|---|---|
| FR-02 | ✅ nav waits for CAL_OK_IMU; CAL_FAIL → FAULT |
| FR-10 | ✅ every traversal counted once (flat-top, back-off, pick-up cases tested) |
| FR-11 | ✅ Σ tilt·Δs method, §3.1 |
| FR-12 | ✅ highest hump per mission, reset on Ready→Running, in the `hump_state` word; 🟡 not in telemetry yet |
| FR-23 / FR-29 | 🟡 tilt, class, hump fields ready in the word; sending to comms_task waits on Buddy 1 |
| FR-27 | 🟡 classifier built and running on the car; thresholds not yet validated under motor power |
| FR-28 | ✅ T = 25°, N = 3; normal humps (up to 15.7°) do not trip; knocks filtered (one residual false fault, §6) |
| NFR-05 | 🟡 revised to ±2 mm / 20 % (limit ±5 mm), self-driven. Hand pushes: every clean pass within target (10 mm 3/3, 12 mm 1/1, 23.5 mm 3/3); limit 10 mm 11/11 (8 earlier + 3 on the final build), 12 mm 2/3, 23.5 mm 3/5. Failures traced to wheel skid or pivoting. Self-driven passes still to run |
| NFR-13 | ✅ cal fail, abnormal tilt, read failure (50 ms); init fail not run |
| NFR-18 | ✅ brake 47–61 µs after STOP_IMU (limit 20 ms) |
| NFR-19 | ✅ 20.5–20.8 ms over 4 trials (limit 40 ms) |
| NFR-11 | ✅ period 9 653–10 352 µs, WCET 1 466 µs (Wi-Fi off); 🟡 Wi-Fi-on repeat pending |
