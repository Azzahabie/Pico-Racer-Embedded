# IMU: abnormal motion and stop behaviour (Buddy 4)

Reference for `imu_task`. Sources: Week 6 requirements (FR-27, FR-28, NFR-13, NFR-18, NFR-19), PostTri1, and the thread with Hanis (Buddy 2) and Buddy 5.
All thresholds marked *proposed* stay PROPOSED until they are set from A4 data.

## 1. Should the car stop on abnormal motion?

Yes, but only for the triggers in section 3. Everything else is classified or reported, and the car keeps going.

## 2. What counts as abnormal motion

Only one thing (FR-28):

> Tilt above **T = 20°** for **N = 3** consecutive samples (T and N proposed).

- A normal hump crossing must **not** meet this condition.
- T is set at least **5° above the steepest measured hump pitch**.
- Samples are 10 ms apart, so 3 samples take about 20 ms.

## 3. Event table

| Event | Definition | Stops the car? | Source |
|---|---|---|---|
| Hump climb / descent | Pitch above θ_on = 5° for 3 samples (onset); within θ_off = 3° of flat for 3 samples (end). Both proposed. | No. Measured and reported. | FR-10 to FR-12 |
| Sudden impact | Acceleration spike above A = 1.5 g (proposed) | **No.** Reported as an event only. Stopping on any impact was rejected (false stops mid-run). | FR-27 |
| Stationary | No change in `distance_state` and encoder turn rate below 5 °/s for 5 samples (proposed) | No | FR-27 |
| Turning | Encoder-derived turn rate above 20 °/s (proposed) | No | FR-27 |
| Accelerating / braking | Motion state from `distance_state` | No. Used to hold pitch while accelerating. | Thread with Hanis |
| **Abnormal motion** | Tilt above T for N = 3 samples | **Yes** | FR-28, NFR-13 |

## 4. IMU fault: what stops the car (NFR-13)

An IMU fault is any of:

| Fault | When | Result |
|---|---|---|
| Initialisation failure | Before the mission | Mission movement prevented, system enters Fault |
| Calibration failure | Before the mission | Mission movement prevented, system enters Fault |
| Abnormal motion (tilt above T for 3 samples) | During a mission | Urgent motor stop and Fault |
| Read failure: no valid sample within **Y = 50 ms** (proposed) | During a mission | Urgent motor stop and Fault |

## 5. Stop sequence during a mission

1. `imu_task` sets `STOP_IMU` on `motion_flg`. This wakes `motion_task` directly, without going through `nav_ctrl_task`.
2. `motion_task` brakes as its first write, within **20 ms** of the request (NFR-18).
3. `imu_task` reports the fault: `NAV_FLG_IMU_FAULT` (0x20) on `nav_flg`, and the `FAULT_IMU` bit (bit 2) in the shared fault mask. `nav_ctrl_task` moves the system to Fault.
4. `STOP_IMU` is cleared **only** by `FAULT_CLEAR(IMU)`, which `nav_ctrl_task` sends after `imu_task` recalibrates successfully.
5. `imu_task` clears its own fault bit when recalibration succeeds.

`imu_task` never writes motor outputs. Only `motion_task` drives the motors.

## 6. Timing (NFR-19)

- `imu_task` must raise the urgent-stop request within **40 ms** of the first sample above T.
- Budget: about 20 ms to see 3 samples at 10 ms, leaving about 20 ms for processing and setting `STOP_IMU`.
- Measure with the RP2040 1 MHz timer (professor-approved method for the team's tests). Confirm the approval covers the NFR-19 test.
- Bench numbers (no motor stop leg) and on-car numbers must be reported separately.

## 7. Interface constants (from master)

| Name | Value | Where |
|---|---|---|
| `NAV_FLG_CAL_OK_IMU` | 0x08 | `nav_flg` |
| `NAV_FLG_CAL_FAIL` | 0x10 | `nav_flg` |
| `NAV_FLG_IMU_FAULT` | 0x20 | `nav_flg` |
| `FAULT_IMU` | bit 2 | shared fault mask |

Fault mask bits: 0 MOTOR_STALL, 1 ENCODER_FAULT, 2 IMU_FAULT, 3 SCAN_SENSOR_LOST, 4 LINE_SENSOR.

## 8. Not on master yet (use marked stubs)

- `fault_set()` / `fault_clear()` and the fault flag ID (Buddy 5 to add).
- `nav_bind()` call in `app_main.c`, `imu_flg`, and the shared `cal_req_bit` (Buddy 5 and Buddy 3).
- `motion_flg`, `STOP_IMU`, `distance_state` (on `buddy2/motion`).
- `hump_state` word in `messages.h`.

## 9. Open caveats

- My extra validity checks (5 identical raw samples in a row; |a| above 12 g) are **not** in the documents. Either record them in PostTri1 as part of "no valid sample", or keep them out of the stop path.
- False stops are the main risk. Hard braking, a fast hump or a bump could briefly look like tilt. A4 data must show that the steepest hump and normal braking stay well below T. If they do not, raise T.
- Highest hump versus last hump in telemetry (`S.hgt`) is still undecided with Buddy 1.
