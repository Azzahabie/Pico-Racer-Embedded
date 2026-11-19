# Final ramp: same height, a different width (19 Nov)

Buddy 4 (IMU). New from the prof's side (19 Nov, via the team): the final
ramp is the same height as before (about the smaller practice ramp; the demo
hump is about 195 x 10 mm) but a **different width**. This page says what
that does to hump detection, which build to flash, how to measure the real
ramp on the day, and the car tests that decide it.

Labels: **M** measured on the car (`terrain_analysis_report.md`), **C**
calculated, **sim** host model: the real `hump.c` fed by a simulated car, as
in `tests/imu_hump` (20 noise seeds per case, +/-4 mg per 100 ms window,
48 mm/s unless said). **Nothing on this page has been run on the car.**

## 1. Width across the track

Height = tilt x encoder distance (report §3.1). Neither depends on how wide
the ramp is, as long as both wheels ride up it together.

| Ramp width W (across) | What happens | Hump detection |
|---|---|---|
| wider than the car's outer tyre span plus the line-following wander on both sides | both wheels climb together, pitch only, as on the practice ramps | unchanged |
| narrower than the gap between the tyres | the wheels straddle it, only the front support climbs, the chassis may scrape | height meaningless: out of scope, look on the car |
| in between, or the car off centre | one wheel climbs and the car rolls: one wheel 10 mm up on the 127 mm track is 79 mg (4.5 deg) of roll (C), counted as tilt | a hump may count, the height is wrong |

The tyre centres are 127 mm apart (M, `docs/motion/results.md`); the outer
tyre span is not in the docs: measure it. The line sensors sit on the
centreline 125 mm ahead of the axle (M), so the ramp's width does not change
how they cross it, as long as the black line runs over the ramp (still an
open question, `diag_plan_v2.md` question 7).

## 2. Length along the track (sim)

A longer ramp of the same height is gentler. A hump starts only after 3
windows (300 ms) tilted at least `HUMP_ENTER_MG` (50 mg, 2.9 deg). The slope
of the way up is about 1000 x h / U mg for height h and way-up length U (C).

**10 mm, up half way and down the other half**, 20 runs each. "In target" =
counted once and within the NFR-05 target (+/-2 mm).

| Length L | Way up U | Slope | Default build | Long-ramp build (`HUMP_ENTER_MG=40`, `HUMP_WAY_DOWN=1`) |
|---|---|---|---|---|
| 195 (demo) | 98 | 103 mg, 5.9 deg | 20/20 in target, 9.6-10.0 mm | the same |
| 300 | 150 | 66 mg | 20/20, 9.6-9.8 | 20/20, 9.6-9.8 |
| 360 | 180 | 55 mg | 20/20, 9.0-9.9 | 20/20, 9.6-9.9 |
| 380 | 190 | 53 mg | 20/20, but 13 runs print a "hump ignored" tail | 20/20, 9.7-10.0 |
| 400 | 200 | 50 mg | 15 once, **5 twice**; 13/20 in target; 5.2-9.9 | 20/20, 9.7-9.9 |
| 420 | 210 | 48 mg | **8 missed**, 1 twice; 5/20 in target | 20/20, 9.7-10.0 |
| 440 | 220 | 45 mg | **0/20 seen** | 20/20, 9.1-9.8 |
| 460 | 230 | 43 mg | 0/20 | 20/20, 9.0-9.9 |
| 480 | 240 | 42 mg | 0/20 | 18/20 in target |
| 500 | 250 | 40 mg | 0/20 | 14/20 in target (6.0-9.9) |
| 540 | 270 | 37 mg | 0/20 | 5/20 seen |
| 560 and longer | 280+ | 36 mg and less | 0/20 | 0/20 |

8 and 12 mm follow the slope, not the length: the default is reliable to
8 mm over 300 mm and 12 mm over 460 mm (both about 53 mg); the long-ramp
build to 8 mm over 380 mm and 12 mm over 560 mm (both about 43 mg).

**Rule of thumb (sim):** default: the way up at most 18 x h long (slope 0.055
or more, 3.2 deg). Long-ramp build: at most 23 x h (0.043, 2.5 deg).

**Level top** (the practice ramp's slopes, 100 mm up and 100 mm down, 10 mm):
default counts **one hump with a top up to 115 mm, two from 120 mm** (125 mm
at 150 mm/s), at every length tested to 800 mm. The hump closes on the top
after 120 mm of level (`HUMP_TOP_MAX_MM`), and the way down then starts a new
"climb". With `HUMP_WAY_DOWN=1`: one hump at every top length, 9.7-10.2 mm.
Up = top = down (L/3 each): default counts twice from L = 400 (top 133);
long-ramp build once up to L = 750.

**Rounded bump** (a cosine profile): the toe and the foot, gentler than the
start threshold, are never added, so it reads low. Default: in target up to
450 mm (8.2-8.7 mm), 8/20 at 500 mm, counted twice in 4/20 runs at 550 mm and
19/20 at 600 mm. Long-ramp build: in target up to 550 mm (8.2-8.5), 11/20 at
600 mm, never counted twice.

**Speed:** at 150 mm/s (the hump speed cap; line following is ~48 mm/s) the
edges move by under 20 mm, and 100 mm long ramps read up to ~1.2 mm lower.

**Approach angle:** the tilt magnitude is the ramp's own slope whatever the
heading, so the angle does not change whether the hump is seen. The distance
over it grows as 1/cos(angle), so the height reads high by that factor (C).
Sim, 10 mm: 20 deg 10.1-10.7, 30 deg 11.1-11.7, **45 deg 13.5-14.3 (out of
target, inside the +/-5 mm limit)**. It fits the 23 mm hump on the bend
reading +4.8 mm, +21 % (M), which 1/cos gives at about 34 deg (C).

**What limits gentle ramps:** `HUMP_ENTER_MG` with its 3-window confirm. Not
the window count (3 windows are 14 mm at 48 mm/s, C) and not `HUMP_MIN_MM`
(once seen, the height is right). "hump ignored: only x mm" shows up only at
the edge: the tail of the way down taken as a new climb.

**Floor tilt near the ramp:** the detector learns level only while stopped
near it. A floor tilted 15 mg (0.9 deg) along the track around the ramp moves
the heights by about 1.5 to 3.5 mm in either build (it is real rise under
the car), 30 mg by 2.5 to 8 mm (sim). The 14/14 demo-hump passes (M) say the demo
floor was not like that; check the floor at the final ramp (section 4).

**Noise:** at +/-15 mg per window the default's edge widens (10 mm over
350 mm counted twice in 3 of 20); the long-ramp build stays 19-20/20 in
target up to 400 mm (sim).

**Model limits:** tilt = the slope under the car, as `tests/imu_hump`. It
gives the demo hump's height as the car did (sim 9.6-10.0 mm, car 8.9-11.3,
mean 9.96, M), but not the car's late close: on the car the demo hump's way
down was never read, so it closed 14-15 cm past the crest by the 120 mm cap
(M); the model reads the way down and closes at the foot. A rigid
two-contact model (front support 100-150 mm ahead of the axle) read the demo
hump at 4-7 mm, which the car did not, so it is not used.

## 3. The build switches (`hump.h`)

The defaults are the car build as driven on 2-3 Nov. A fresh default build
gives a UF2 byte-identical to the branch before the switches were added.

| Switch | Default | Long ramp | What it does | Cost |
|---|---|---|---|---|
| `HUMP_ENTER_MG` | 50 | 40 | a climb from 40 mg (about 2.3 deg) starts a hump | 10 mg over the floor spots (up to 30 mg, M) instead of 20. Sim, worst case: a floor tilted 30 mg for the whole run with +/-10 to 15 mg noise gave a false hump in 2-19 of 40 runs at 40 and none at 50 (none in either with +/-4 mg). Needs flat-floor run F1 first |
| `HUMP_WAY_DOWN` | 0 | 1 | once a hump closes while driving, downhill is its way down until the next climb, not a new hump. Prints `[imu] hump: the rest of its way down, not a new hump` once | the hump state reads NONE on that rest of the way down, so motion's hump allowances (the doubled line-lost time) do not cover it. A hump whose way up was never seen is read from its far side (as now), and the next hump then from its far side too (test "unseen way up, steep far sides") |

Build with the team's kit: `-DHUMP_ENTER_MG=40 -DHUMP_WAY_DOWN=1` inside
`GCC=`, as for the other `#ifndef` switches (`diag_plan_v2.md` §2.2). A
switched image prints at boot:
`[imu] humps: start at 40 mg (2.2 deg), way-down rule on`
(the default image prints no such line). Check it before the run.

## 4. On the day: measure the ramp, choose the build

Measure and write down (ruler and tape):

1. **h**: height at the highest point, from the mat (ruler, +/-0.5 mm).
2. **U**: length of the way up, from where it leaves the mat to where it
   reaches h. **D**: the way down, the same way. **P**: the level top (0 if
   it peaks).
3. Rounded ramp: the steepest part of the way up, as rise over 50 mm there.
4. **W**: width across, and the car's outer tyre span.
5. Where it sits: on a straight or a bend, and the angle the line crosses it.
6. Floor: put the car still on the mat ~20 cm before the ramp and just after
   it. A `nose` or `leaning` word in the status line (5 deg or more), or, in
   an `IMU_VERBOSE` image, axes more than ~15 mg off the calibration spot,
   means floor tilt will add to the error (sim, section 2).

Slope s = h / U (C). Then:

| Measured | Flash |
|---|---|
| s at least 0.055 (10 mm: U up to 180 mm) and P up to 100 mm | default |
| s at least 0.055 and P over 100 mm | `HUMP_WAY_DOWN=1` |
| s from 0.043 to 0.055 (10 mm: U 180-230 mm), any P | `HUMP_ENTER_MG=40 HUMP_WAY_DOWN=1`, after F1 passes |
| s under 0.043 (10 mm: U over 230 mm) | out of reach (sim): say so in the demo; do not go below 40 (floor spots 30 mg) |
| W narrower than the tyre span | out of scope: look on the car |
| crossed at more than ~30 deg | expect +15 % or more (C) |

Within 0.005 of a boundary, take the row below it.

## 5. Car test matrix

Every row: 5 self-driven crossings (NFR-05), line following, car calibrated on
flat floor, console logged (USB, or netlog for debugging). Lift the car only
after the `===== HUMP` line (report §5.2).

Console lines to look for:

- `[imu] hump: climbing (<n> deg up, <n> mm)`: about 15 mm onto the way up
  (3 windows at 48 mm/s, C)
- `[imu] hump: descending (<n> mm at the top, <n> deg)`: if the way down is
  read
- `[imu] ===== HUMP <n>: <x.y> mm high =====  (up .., down .., steepest <n>
  deg, <n> mm long)`: once per crossing. "steepest" is the slope the car saw;
  "mm long" should be about the ramp's length plus up to ~120 mm
- `[imu] hump ignored: only <x.y> mm high`: should not appear
- `[imu] hump: the rest of its way down, not a new hump`: `HUMP_WAY_DOWN=1`
  only
- status line: `| on a hump: climbing, <n> mm up`, then `| humps: <n>
  (highest <n> mm)`

| # | Ramp (along the track) | h | Approach | Speed | Build | Expected per crossing | Pass |
|---|---|---|---|---|---|---|---|
| F1 | none: flat course mats, 1.2 m or more with the bend | 0 | line | 48 | each build to be used | no `hump: climbing`; status `humps: none yet` | 3/3 runs with 0 humps |
| R1 | the demo hump, 195 mm (control, as driven 1-3 Nov) | 10 | square | 48 | default | `climbing (5-7 deg up)`, one `HUMP n: 8.0-12.0 mm`, about 14-15 cm past the crest (M) | 5/5 once, within +/-2 mm |
| R2 | the final ramp, s 0.055 or more, P up to 100 | as ruled | square | 48 | default | `climbing`, maybe `descending`, one `HUMP` line by ~15 cm past the far end; no `ignored` | 5/5 once, within +/-2 mm (20 % if over 10 mm) |
| R3 | as R2 | as ruled | on the bend or at its real angle | 48 | as R2 | as R2; reads high by 1/cos(angle) | 5/5 once; within the +/-5 mm limit |
| R4 | the final ramp with P over 100 | as ruled | square | 48 | default, once | (sim) `HUMP 1` on the top, then `hump: climbing` on the way down and `HUMP 2` past the foot: confirms the switch is needed | seen once is enough |
| R5 | as R4 | as ruled | square | 48 | `HUMP_WAY_DOWN=1` | `HUMP 1` on the top, then `the rest of its way down, not a new hump`; no second `HUMP` | 5/5 once, within target |
| R6 | the final ramp, s 0.043-0.055 | as ruled | square | 48 | default, once | (sim) no `climbing` line, or a low `HUMP` and a second `HUMP` or `ignored` | seen once is enough |
| R7 | as R6 | as ruled | square and bend | 48 | `HUMP_ENTER_MG=40 HUMP_WAY_DOWN=1` (after F1) | `climbing (2-3 deg up)`, one `HUMP` | 5/5 once, within target |
| R8 | the final ramp, stopped on it by a hand, then off | as ruled | square | 48 | the build chosen | `HUMP n` ("stopped on top" or closed on the top), then `on level ground again` or `the rest of its way down`; no second `HUMP` | 1 hump |
| R9 | only if the base speed is raised | as ruled | square | up to 150 | the build chosen | as R2 | as R2 |

Record for every crossing: the `HUMP` line (height, steepest, mm long), any
`descending` / `ignored` / `rest of its way down` line, the floor readings
from section 4, and the image's boot line. If the car disagrees with the
model, an `IMU_CSV` image gives the raw samples to replay through
`tests/imu_hump`.
