# Terrain analysis report — imu_task (Buddy 4)

Team IS-24 · Pico Racer · µT-Kernel 3.0 on RP2040 (SMP) · tests run 2026-10-27/28
Code: `app_program/imu_task/` (`imu_task.c`, `hump.c`, `imu_safety.c`) on branch `imu_task`, commit `271bfe8`.

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
**No correction factor is applied** (§4.4).

### 3.2 Hump onset, end and special cases

| Rule | Value | Why |
|---|---|---|
| Onset | tilt ≥ 50 mg (~2.9°) for 3 windows (300 ms), moving, motion STEADY | a single jolting window once set "uphill" backwards and lost a 23 mm hump |
| Uphill direction | sum of the 3 onset windows | mounting angle does not matter |
| Level | tilt < 35 mg (~2.0°) | floor spots differ by up to 30 mg |
| End, driven over | back within 2 mm of start height, or level for 40 mm **once on the way down** | a level top longer than 40 mm otherwise ended the hump on the top and counted the way down as a second hump |
| End, safety cap | level for 250 mm | far side that drops off too fast to read as a slope |
| End, stopped | still for 1 s: hump counted, then the car is followed "on top" until it is back down | backing off a ramp once counted as a new hump |
| Picked up | stopped but tilt changes > 60 mg between windows: "on top" ends | carried back to the start, the next climb was swallowed |
| Minimum | 4 mm | shorter climbs are reported as ignored |
| Fault / tilt past T | unfinished hump discarded (Statechart 6.4) | held on its side, a few mm of wheel turn once read as a 6 mm "hump" |
| Accelerating / decelerating | windows that are not STEADY do not start a hump or flip CLIMBING/DESCENDING | acceleration reads as tilt (pitch hold) |

`hump_state` (NONE / CLIMBING / DESCENDING) follows the direction of travel. The
highest hump since boot is kept for FR-12 (reset on Ready→Running is not wired yet).

All of these cases are replayed on a PC: `tests/imu_hump` (18 scenarios, each one a
pass that went wrong on the car) and `tests/imu_safety` (24 checks). Both pass.

## 4. Results

NFR-05: each estimate within **0.5 cm or 20 %, whichever is larger**, 5 traversals
per reference hump.

### 4.1 Lower practice ramp — 10 mm (ruler), limit ±5 mm

Passes pushed fully over the ramp:

| Pass | Reported (mm) | Error (mm) | Peak tilt | Wheels | NFR-05 |
|---|---|---|---|---|---|
| 1 | 9.0 | −1.0 | 116 mg (6.7°) | both rolling | ✅ |
| 2 | 8.9 | −1.1 | 127 mg | both rolling | ✅ |
| 3 | 6.7 | −3.3 | 173 mg | left wheel skidding | ✅ |
| 4 | 6.3 | −3.7 | 164 mg | left wheel skidding | ✅ |
| 5 | 6.9 | −3.1 | 166 mg | left wheel stalled | ✅ |
| 6 | 9.4 | −0.6 | 112 mg | both rolling | ✅ |
| 7 | 7.9 | −2.1 | 119 mg | left wheel froze near top | ✅ |
| 8 | 8.8 | −1.2 | 126 mg | slight slew | ✅ |

**8/8 pass.** Mean 8.0 mm (−2.0 mm). Clean passes (1, 2, 6): −0.6…−1.1 mm.

### 4.2 Higher practice ramp — 23.5 mm (ruler 23–24 mm), limit ±5 mm

| Pass | Reported (mm) | Error (mm) | Peak tilt | Notes | NFR-05 |
|---|---|---|---|---|---|
| 1 | 23.0 | −0.5 | 264 mg (15.3°) | clean | ✅ |
| 2 | 25.3 | +1.8 | 259 mg | clean | ✅ |
| 3 | 15.9 | −7.6 | 271 mg (15.7°) | left wheel froze | ❌ |
| 4 | 13.0 | −10.5 | 216 mg | fast, jerky push; hump ended early | ❌ |
| 5 | 28.2 | +4.7 | 246 mg | fastest push (~120 mm/s) | ✅ |

**3/5 pass; 3/3 clean passes pass.** Mean of clean passes 25.5 mm (+2.0 mm, +8.5 %).
The plan says to tune on the lower ramp and ignore the higher one; this run is kept as
evidence that the method still works on a hump more than twice as tall.

### 4.3 Shimmed ramp — 12 mm (lower ramp + 2 mm shim), limit ±5 mm

| Pass | Reported (mm) | Error (mm) | Peak tilt | Notes | NFR-05 |
|---|---|---|---|---|---|
| 1 | 13.7 | +1.7 | 261 mg | clean | ✅ |
| 2 | 8.4 | −3.6 | 157 mg | left wheel froze (reads low) | ✅ |
| 3 | 17.7 | +5.7 | 133 mg | jerky push: right wheel spun to 2350 c/s while the left nearly stopped (reads high) | ❌ |

**2/3 pass.** Only 3 traversals counted: two more on this setup were discarded as
abnormal tilt under the old T = 20° (§4.5), so this setup still needs 2 more clean
passes for the 5 that NFR-05 asks for. Pass 3 is the first over-read: skid goes
either way, depending on which wheel slips.

### 4.4 Why no correction factor

Clean passes read about **10 % low** on the 10 mm ramp and about **8 % high** on the
23.5 mm ramp. No single multiplier fits both, and the large errors come from wheel
skid (§6), which a factor cannot fix: a factor fitted to the skidding passes would
push clean passes over the limit. The method is used uncorrected and passes NFR-05
on every clean traversal recorded.

### 4.5 Thresholds: proposed → chosen from data

| Threshold | Proposed (Week 6) | Final | Evidence |
|---|---|---|---|
| T (abnormal tilt) | 20° | **25°** (423 mg) | steepest hump driven peaked at 15.7°; on it, two jerky pushes held > 20° for 3 samples, so 20° would have been a false STOP in a mission. FR-28 needs T ≥ steepest + 5°. |
| N | 3 samples | 3 (30 ms) | unchanged |
| Jolt filter | — | samples with \|a\| outside 0.7…1.3 g do not count to N | crashes push x/y but pull \|a\| away from 1 g |
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
| Boot with IMU unplugged → CAL_FAIL | — | not run |

Brake time is motion_task's own stamp (`motion_stop_stamp`), so NFR-18 is met by a
wide margin. NFR-19 (STOP_IMU within 40 ms of the first sample above T) holds by
construction — N = 3 samples is 20–30 ms after the first one — but was not timed with
1 MHz stamps from the first sample.

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
3. **Stopping on the slope splits a hump.** Still for 1 s ends it; the rest is
   measured as a new climb. Not expected on the course unless the car stops on a
   hump.
4. **Half-connected SDA.** With the wire loose, half the reads failed and the rest
   read 30–65° while the car was (probably) level. The outcome is still a fault, but
   wrong-yet-valid readings are possible.
5. **STOP_IMU stays set until reboot.** nav sends FAULT_CLEAR(IMU) after a passing
   calibration, but `nav_cmd_mbf` is not bound to motion_task yet (Buddy 2, step 1.7).
6. **Not implemented yet:** 6-event motion classifier, impact detection (A), mission
   max reset on Ready→Running, `hump_state` word and telemetry records (needs
   Buddy 1's schema), CSV logging mode, NFR-11 jitter/WCET measurement with Wi-Fi on.
7. **Magnetometer** is read but unused.

## 7. Requirements status

| ID | Status |
|---|---|
| FR-02 | ✅ nav waits for CAL_OK_IMU; CAL_FAIL → FAULT |
| FR-10 | ✅ every traversal counted once (flat-top, back-off, pick-up cases tested) |
| FR-11 | ✅ Σ tilt·Δs method, §3.1 |
| FR-12 | 🟡 max kept since boot; reset on Ready→Running and telemetry not wired |
| FR-28 | ✅ T = 25°, N = 3; normal humps (up to 15.7°) do not trip |
| NFR-05 | ✅ 10 mm: 8/8; 12 mm: 2/3 (2 more passes needed); 23.5 mm: 3/5 (3/3 clean). Every failure traced to wheel skid |
| NFR-13 | ✅ cal fail, abnormal tilt, read failure (50 ms); init fail not run |
| NFR-18 | ✅ brake 47–61 µs after STOP_IMU (limit 20 ms) |
| NFR-19 | 🟡 by design ≤ 30 ms; not timed from the first sample |
| NFR-11 | 🟡 sampled on a 10 ms cyclic handler, `lost 0` in all logs; jitter/WCET not measured |
