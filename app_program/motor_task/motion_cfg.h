/*
 *  motion_cfg.h
 *  Every tunable for motion_task (Buddy 2) in one place.
 *
 *  CONFIRMED = from the approved team pin map (28 Sep 2026), the course
 *  material or the RP2040 datasheet.  PROPOSED = a starting value, to be
 *  replaced by what the bench test measures (docs/motion/README.md).
 */

#ifndef MOTION_CFG_H
#define MOTION_CFG_H

/* ------------------------------------------------------------------ *
 *  Pins  (CONFIRMED, approved pin map)
 * ------------------------------------------------------------------ */

#define MOTION_PIN_M1A (8u)  /* left motor input A, PWM4 A  */
#define MOTION_PIN_M1B (9u)  /* left motor input B, PWM4 B  */
#define MOTION_PIN_M2A (10u) /* right motor input A, PWM5 A */
#define MOTION_PIN_M2B (11u) /* right motor input B, PWM5 B */

#define MOTION_PIN_ENC_L_A (0u) /* Grove 1 */
#define MOTION_PIN_ENC_L_B (1u)
#define MOTION_PIN_ENC_R_A (2u) /* Grove 2 */
#define MOTION_PIN_ENC_R_B (3u)

/* Bench test switch.
   0 (the default, and the only value ever committed): integration build.
     GP20 is never read or configured (it is Buddy 5's start button) and
     nothing can start the motors.  Encoders, the stall check and the
     status lines still run.
   1: bench image for the tests in docs/motion/README.md, wheels off the
     ground.  Set it here locally, rebuild, and never commit it. */
#ifndef MOTION_BENCH
#define MOTION_BENCH (0)
#endif

/* Minimum-bit switch (changed requirement, Tribunal 1).
   1 (the default, and the only value ever committed): packed layouts.
   0: the Week 6 (unpacked) layouts, kept only to measure the Week 13
     "before" (RAM, message sizes, timing) on the same code.
     tests/week13/measure.sh builds both; make -C tests tests both. */
#ifndef MOTION_PACKED
#define MOTION_PACKED (1)
#endif

#if MOTION_BENCH
/* Robo Pico user button, used only by the bench test: press to start a
   test, press again to abort.  Read by polling, never by interrupt. */
#define MOTION_PIN_BUTTON (20u)
#endif

/* ------------------------------------------------------------------ *
 *  Polarity  (set these from bench test steps 1 and 2)
 *
 *  MOTOR_x_INVERT: 1 if the wheel turns backward on a "fwd" step.
 *  ENC_x_INVERT:   1 if the count goes down when the wheel turns forward.
 * ------------------------------------------------------------------ */

#define MOTION_MOTOR_L_INVERT (0)
#define MOTION_MOTOR_R_INVERT (1)
#define MOTION_ENC_L_INVERT   (0)
#define MOTION_ENC_R_INVERT   (1)

/* ------------------------------------------------------------------ *
 *  PWM  (CONFIRMED: clk_sys 125 MHz in cpu_clock.c; driver limit 20 kHz)
 * ------------------------------------------------------------------ */

#define MOTION_SYS_CLK_HZ (125000000u)
#define MOTION_PWM_HZ     (10000u) /* PROPOSED, design 10 kHz */
#define MOTION_PWM_TOP    ((MOTION_SYS_CLK_HZ / MOTION_PWM_HZ) - 1u)

/* Duty is signed per-mille: +1000 full forward, -1000 full reverse,
   0 brake (both inputs low). */
#define MOTION_DUTY_FULL (1000)

/* ------------------------------------------------------------------ *
 *  Task and timing  (Section 3 of the design: priority 11, core 1, 10 ms)
 * ------------------------------------------------------------------ */

#define MOTION_TASK_PRI   (11)
#define MOTION_TASK_STACK (4096) /* same as every other task (SMP) */
#define MOTION_TICK_MS    (10u)

/* No tick for this long means the cyclic handler has stopped while the
   kernel timer still runs: brake and count a lost tick.  It must stay
   below MOTION_WDT_MS, or the watchdog resets the chip first and this
   brake never runs (it was 50 ms, W8; checked in motor_task.c).  Two
   tick periods: the longest period seen on the car is 11.5 ms
   (docs/INTEGRATION.md, NFR-11).  If the kernel timer itself stops, this
   timeout stops with it and only the watchdog acts. */
#define MOTION_TICK_TIMEOUT_MS (20)

#define MOTION_REPORT_TICKS (20u) /* status line every 200 ms */

/* Hardware watchdog (prof Week 6 item 2, PreTri1: 30 ms PROPOSED), fed
   at the end of every completed tick.  It must stay above the longest
   gap between feeds measured on the car, which the timing line prints. */
#define MOTION_WDT_MS (30u)

/* FR-30 stall detection  (PROPOSED: [t_stall] 300 ms, [N_min] 3 counts) */
#define MOTION_STALL_TICKS      (30u)
#define MOTION_STALL_MIN_COUNTS (3)

/* Edges one phase must make while the other stays still before a stall is
   called an encoder fault rather than a motor stall.  PROPOSED: raise it
   above any chatter seen with the wheel held (bench test 1). */
#define MOTION_STALL_ENC_EDGES (3u)

/* Below this duty a wheel may legitimately not move (dead band), so the
   stall check is skipped.  PROPOSED: replace with the ramp-test result. */
#define MOTION_DEADBAND_DUTY (150)

/* Reverse (statechart REVERSING, scan_task's REVERSE command): PID target
   for both wheels, counts per tick, driven backwards.  PROPOSED: 10 counts
   per tick, about 8 cm/s (calculated: 10 x 0.079 mm every 10 ms), so
   scan_task's 150 mm reverse takes about 2 s, inside its 5 s timeout. */
#define MOTION_REV_TARGET (10)

/* Line following (statechart LINE_FOLLOW, Buddy 3's line_state word).
   Both wheels run under the PID at MOTION_LF_BASE; a correction moves the
   two targets apart, + = turn left.  The correction is in 1/100 counts per
   tick (MOTION_LF_SCALE), built each tick from the line error e
   (-100..+100):
     target = e x MOTION_LF_GAIN + (e - e MOTION_LF_DAMP_TICKS ago) x
              MOTION_LF_DAMP
   The second term is the damper: while the car swings back toward the
   line the error falls, so it takes correction off before the car
   overshoots.  The target is capped at MOTION_LF_SOFT_MAX, and the
   applied correction moves toward it by at most MOTION_LF_SLEW a tick, so
   it eases in and out.  Only an error of at least MOTION_LF_HARD_ERR held
   for MOTION_LF_HARD_TICKS gets the hard turn, MOTION_LF_HARD_MAX: the
   inner wheel turns back at 1 count per tick, a near pivot (never 0, so
   the stall check still sees it move).
   PROPOSED, tune on the floor.  S4 (30 Oct): base 15 (about 12 cm/s) ran
   off the line, 8 (about 6 cm/s) next.  31 Oct: 5 (about 4 cm/s) with
   gain 3 strayed within a few cm, so slower again and stronger: 4 is
   about 3.2 cm/s (calculated, 0.079 mm a count), half error already gives
   2.5 counts, and the soft cap leaves the inner wheel at 1 count.
   1 Nov (user: faster): base 4 -> 6 (about 4.7 cm/s), now that the
   adaptive thresholds and the last-seen steering are in.  The gain and
   damper scale with the base (tuned 5 and 15 at base 4), so a given
   error turns the car on the same curve at any base speed. */
#ifndef MOTION_LF_BASE
#define MOTION_LF_BASE        (6)
#endif
#define MOTION_LF_SCALE       (100)
#ifndef MOTION_LF_GAIN
#define MOTION_LF_GAIN        ((5 * MOTION_LF_BASE) / 4)  /* 7 at base 6  */
#endif
#ifndef MOTION_LF_DAMP
#define MOTION_LF_DAMP        ((15 * MOTION_LF_BASE) / 4) /* 22 at base 6 */
#endif
#define MOTION_LF_DAMP_TICKS  (5u)  /* change measured over 50 ms        */
#define MOTION_LF_SOFT_MAX    ((MOTION_LF_BASE - 1) * MOTION_LF_SCALE)
#ifndef MOTION_LF_SLEW
#define MOTION_LF_SLEW        (50)  /* 0 to SOFT_MAX in 60 ms            */
#endif
#define MOTION_LF_HARD_ERR    (80)
#define MOTION_LF_HARD_TICKS  (25u) /* 250 ms                            */
#define MOTION_LF_HARD_MAX    ((MOTION_LF_BASE + 1) * MOTION_LF_SCALE)
#define MOTION_LF_SEEN_MS     (30u) /* dark below this: line still seen */
#define MOTION_LF_FOUND_TICKS (3u)  /* seen this many ticks: found      */

/* Experiment switch (L4, 19 Nov): GP26 and GP27 do not reliably both sit
   inside the 18 mm line, so a car crossing it can read a short 00 (both
   white) between 01 and 10 (team count of the 4 Nov logs,
   tests/scan_bench/logs: 74 of 467 crossings), and Line Following then
   steers at the full +-100 toward the last-seen side.  Above 0, the full
   error is used only once line_state's dark time has reached this many
   ms (10 ms steps); before that it steers by the word's own error.
   0, the default: the full error from the first 00, as before.  A car
   tuning guess (about 50 was suggested); set it with -D, never commit
   it changed without car runs. */
#ifndef MOTION_LF_LOST_DIR_MS
#define MOTION_LF_LOST_DIR_MS (0u)
#endif

/* Line search (statechart LINE_SEARCH, [L_search]).  Both sensors sit
   inside the 18 mm line, so a centred car reads 11 and only a car off the
   line reads 00: after LINE_LOST_MS of 00 the car pivots on the spot
   toward the side the line was last seen (|error| >= MOTION_LS_DIR_MIN
   while a sensor saw it; the sign already includes LINE_STEER_INVERT; Line
   Following steers to the same side at full error during that 00), up to
   MOTION_LS_SWEEP_DEG,
   then back past the start to the same angle the other side.  The line
   under a sensor ends it (LINE_FOUND); a full sweep without it is
   LINE_LOST.  PROPOSED: 6 counts per tick per wheel, about 5 cm/s. */
#define MOTION_LS_SPEED     (6)
#ifndef MOTION_LS_SWEEP_DEG
#define MOTION_LS_SWEEP_DEG (45)
#endif
#ifndef MOTION_LS_DIR_MIN
#define MOTION_LS_DIR_MIN   (10)
#endif

/* ------------------------------------------------------------------ *
 *  Car geometry and distance_state  (motion_ds.h)
 * ------------------------------------------------------------------ */

/* Turning (statechart TURNING, FR-08).  nav sends JUNCTION as soon as
   ir_sense_task flags the junction under GP7.  With the barcode-armed rule
   (LINE_JUNCTION_RULE 1, the default in
   proposal/line-barcode-beside-line) that is about 10 mm into the
   crossbar after a valid barcode (LINE_ARMED_CONFIRM_MM of travel; at
   base 4 the length rule comes first, 280 samples = 8.9 mm, C).
   Otherwise (nothing armed, or rule 0) it is once GP7 has been on the
   crossbar LINE_JUNCTION_SAMPLES (80 here, 280 on that branch: 3.8 or
   13.3 mm at base 6, C).  From that point the car drives
   MOTION_TURN_FWD_MM straight so the axle is over the crossing ([d_sensor]:
   GP7 to the wheel axle), and pivots on the spot at MOTION_TURN_SPEED
   until each wheel has turned PIVOT_COUNTS(90) (left, right) or (180)
   (U-turn, counter-clockwise).  Straight on drives MOTION_TURN_CROSS_MM
   ([L_cross]) from the same point across the crossing, ignoring the line.
   Then MOVE_DONE, and Line Following if the line is under the sensors,
   else Line Search.
   PROPOSED: measure d_sensor (GP7 to the axle) on the car; turn speed as
   the line search. */
#ifndef MOTION_TURN_FWD_MM
#define MOTION_TURN_FWD_MM    (135)
#endif
#ifndef MOTION_TURN_WIN_DEG
#define MOTION_TURN_WIN_DEG   (35)   /* last deg of a pivot watching for the new line */
#endif
#ifndef MOTION_TURN_OVER_DEG
#define MOTION_TURN_OVER_DEG  (25)   /* past the target without the line: stop */
#endif
#ifndef MOTION_TURN_IMU
#define MOTION_TURN_IMU       (0)    /* 1: the pivot stops on the IMU, lined up with the line */
#endif
#define MOTION_TURN_CROSS_MM  (40)
#define MOTION_TURN_FWD_SPEED (MOTION_LF_BASE)
#define MOTION_TURN_SPEED     (6)

/* Bypass (statechart BYPASS, FR-18; Integration Rules D14), by encoder
   count: a rectangle with side probes, so an obstacle of unknown size is
   passed (S7 attempts 1-2: fixed 45 deg legs hit long boxes, went wide
   and came back to the line late).  Turns: + = toward the bypass side.
     1. Pivot MOTION_BYPASS_SIDE_DEG out, look along the sidestep (lane
        check, below), drive offset / sin SIDE_DEG, pivot back.  All of
        it happens behind the obstacle (>= 255 mm ahead of the axle).
     2. Drive alongside, MOTION_BYPASS_FIRST_MM, then MOTION_BYPASS_STEP_MM
        at a time.  After each step: pivot 45 toward the obstacle, set
        SCAN_FLG_LOOK so scan_task looks with the servo 45 the same way
        (square-on to its side: at 45 deg incidence the box reflects the
        sound away, S7 echo test 1 Nov), pivot 45 back.  A reading below
        MOTION_BYPASS_SEEN_MM, no echo, or no answer within
        MOTION_BYPASS_LOOK_TICKS = still there.
     3. Passed = clear after it was seen once, or clear and never seen
        after MOTION_BYPASS_UNSEEN_MM alongside (shallower than a step).
        Then MOTION_BYPASS_CLEAR_MM more: a clear look means the far face
        is behind axle + 110 - 29 = axle + 81 (the beam cone at 108 mm);
        stepping in at 70 deg needs the axle only far face + 38 (margin
        included), so 81 + 38 = 119 -> 120 (was 200 for a 90 deg step).
     4. Pivot SIDE_DEG in, drive toward the line until GP26/GP27 see it
        (at most the leg + MOTION_BYPASS_IN_EXTRA_MM), MOTION_LINE_SENSOR_MM
        more so the axle is on the line, pivot SIDE_DEG onto the heading.
   Still there after MOTION_BYPASS_MAX_PASS_MM alongside: give up into
   Line Search (FR-18 "otherwise stop": it ends in LINE_LOST).
   A stop on the way brakes it; it carries on if the stop clears with no
   new command, and a new BYPASS from scan_task starts over.
   The probe pivots happen beside the obstacle, so C_pass 218 (D12) keeps
   the 168 mm corner sweep clear of it. */
/* S7 attempt 3 (1 Nov): the 90 deg pivots turned about 100 deg at the
   bypass pivot speed (user, by eye), so bypass pivots are scaled to 90 %
   of their encoder count.  The sidestep and the turn back in are
   MOTION_BYPASS_SIDE_DEG (70, user suggestion) instead of 90: the leg is
   offset / sin 70 (MOTION_BYPASS_SIDE_LEG_X1000), and stepping in at 70
   needs the axle only far face + 38 mm, not + 113 (calculated).
   Before the sidestep the car looks along it (servo 0): anything closer
   than the leg + MOTION_BYPASS_LANE_MARGIN_MM (front 155 + 50) and it
   pivots back to its heading and sets SCAN_FLG_SIDE_BLOCKED, so
   scan_task re-plans the other side from its original profile instead
   of re-scanning sideways (attempt 3: right side blocked out of the
   scan's view).  No new command within MOTION_BYPASS_SWITCH_TICKS: Line
   Search (it ends in LINE_LOST). */
#define MOTION_BYPASS_PIVOT_PCT       (90)
#define MOTION_BYPASS_SIDE_DEG        (70)
#define MOTION_BYPASS_SIDE_LEG_X1000  (1064)  /* 1 / sin 70 deg, x 1000 */
#define MOTION_BYPASS_LANE_MARGIN_MM  (205)
#define MOTION_BYPASS_SWITCH_TICKS    (500u)  /* 5 s */
#define MOTION_BYPASS_FIRST_MM    (250)
#define MOTION_BYPASS_STEP_MM     (100)
#define MOTION_BYPASS_UNSEEN_MM   (450)
#define MOTION_BYPASS_CLEAR_MM    (120)
#define MOTION_BYPASS_MAX_PASS_MM (1500)
#define MOTION_BYPASS_IN_EXTRA_MM (150)
/* "Seen": the box read 230-333 mm square-on and clear looks 966-1551 mm
   on the car (S7 attempt 3).  At 300 a 333 mm box reading counted as
   clear, so the box was never "seen" and the car stepped on to
   MOTION_BYPASS_UNSEEN_MM: one look too many and an overshoot. */
#define MOTION_BYPASS_SEEN_MM     (450)
#define MOTION_BYPASS_LOOK_TICKS  (300u)  /* 3 s with no answer: still there */
#define MOTION_LINE_SENSOR_MM     (125)   /* GP26/GP27 to the axle, measured 1 Nov */
/* Raised 1 Nov (user OK): legs 10 -> 12 counts per tick, pivots 6 -> 10. */
#ifndef MOTION_BYPASS_SPEED
#define MOTION_BYPASS_SPEED       (12)
#endif
#ifndef MOTION_BYPASS_TURN_SPEED
#define MOTION_BYPASS_TURN_SPEED  (10)
#endif

/* IMU heading reference for the bypass (4 Nov 2026, Buddy 5; see
   docs/motion/right_encoder_scaling.md section 10).  Line following
   zig-zags +-20-30 deg, so the car stops for the box at an angle to the
   line, and an encoder-only rectangle drawn from that pose missed the line
   on the way back (car runs, 4 Nov).  With a valid heading:
   - the line's direction is a running average of imu_task's heading
     (ev_heading_deg) over Line Following, MOTION_HD_TAU_TICKS long,
     trusted after MOTION_HD_MIN_TICKS, restarted on a jump of more than
     MOTION_HD_JUMP_DEG in one tick (the heading resets at mission start),
     forgotten in Idle, Turning and on an IMU fault;
   - the sideways distance from the line is summed every tick off the line
     (distance x sin(heading - line));
   - the sidestep ends at the planned sideways distance; PARALLEL,
     PROBE_OUT and ALONG turn to the line's direction and TURN_IN to
     SIDE_DEG off it, each stopping MOTION_HD_LEAD_DEG early for the
     overshoot, with 2x the encoder count as a backstop;
   - STEP_IN also ends when the line sensors are back over the line's
     position.
   Without a valid heading every phase runs on the encoders as before.
   MOTION_HD_ENABLE 0 builds the encoder-only bypass. */
#ifndef MOTION_HD_ENABLE
#define MOTION_HD_ENABLE      (1)
#endif
/* The average is over distance, not time (changed 4 Nov after trials 9-10):
   at ~4.75 cm/s a 1.5 s average covered only ~7 cm, and line following
   wanders slowly (2 deg right to 28 deg left over 8 s before the box), so
   the line direction came out ~20 deg off, the probes missed the box, the
   car turned in onto it and missed the line.  Weighted by distance over
   MOTION_HD_LINE_MM, a full sway averages out (calculated: +-20 deg over a
   ~380 mm sway -> ~+-3 deg). */
#ifndef MOTION_HD_LINE_MM
#define MOTION_HD_LINE_MM     (400)  /* average over this much travel    */
#endif
/* Snap (20 Nov 2026, car): the 400 mm average lagged a curve.  After a 73
   deg left curve and ~250 mm of straight the line direction was still ~40
   deg off (log: bypass OUT at -41/-42/+34/-52 deg "to the line" with the
   car on the line), so every bypass turned 40 deg too far and ran off the
   play area.  In Line Following, a heading more than MOTION_HD_SNAP_DEG off
   the average is a curve, not a wobble: the average jumps to it (its
   travel count is kept).  0 = off (as before). */
#ifndef MOTION_HD_SNAP_DEG
#define MOTION_HD_SNAP_DEG    (0)
#endif
#define MOTION_HD_SNAP_MM     (60)   /* off by more than SNAP_DEG this far */
/* Trusted after MOTION_HD_MIN_MM of Line Following.  300 -> 100 on 4 Nov:
   on the test track the box sits ~25 cm into the line, so every bypass
   fell back to the encoders and the right ones circled the box; the
   average starts at the heading where Line Following began, so with the car
   placed along the line 100 mm is enough, and better than no heading. */
#define MOTION_HD_MIN_MM      (100)  /* trusted after this much travel   */
#define MOTION_HD_JUMP_DEG    (30)
#define MOTION_HD_LEAD_DEG    (4)
/* STEP_IN with the IMU: if the sensors have not seen the line by the time
   they are over its estimated position, drive this much further first (the
   line was seen 4-30 mm before the estimate in trials 1-7; trials 9-10
   stopped exactly on the estimate and never crossed it). */
#define MOTION_HD_PAST_MM     (80)

/* Known-obstacle bypass (4 Nov 2026, user request): the demo box is known,
   90 mm wide x 130 mm long x 190-200 mm high, so the car does not probe
   beside it (each probe pivot swept the front corner 168 mm out, which is
   why C_pass, and so the sidestep, had to be 248 + edge = 290-365 mm).
   Instead (scan_task sizes the sidestep from the real width, see
   SL_KNOWN_W_MM):
     sidestep to the planned offset -> parallel -> straight on until the
     axle is MOTION_KNOWN_PASS_MM along the line from where it braked ->
     turn in -> look along the way back (IN_LOOK) -> clear: back to the line
     by the measured sideways distance; not clear: turn back parallel, on
     MOTION_KNOWN_MORE_MM, look again, up to MOTION_KNOWN_MORE_MAX_MM.
   PASS: stop distance (<= 252 mm seen, 4 Nov) + sensor to axle 155 + box
   130 + rear corner swing when turning in sqrt(70^2 + 63^2) = 94 + gap 60
   = 691 -> 700 mm (calculated).  The way back counts as blocked for a
   reading under MOTION_KNOWN_IN_LOOK_MM (scan_task's lane check also
   reports a lane shorter than leg + D_stop + 50 as 1 = blocked).
   0 = the probing bypass (D14) for obstacles of unknown size. */
#ifndef MOTION_BYPASS_KNOWN
#define MOTION_BYPASS_KNOWN       (0)
#endif
#define MOTION_KNOWN_PASS_MM      (700)
#define MOTION_KNOWN_MORE_MM      (100)
#define MOTION_KNOWN_MORE_MAX_MM  (400)
#define MOTION_KNOWN_IN_LOOK_MM   (300)

/* Arc bypass (4 Nov 2026, user request; test option, 0 = zig-zag): round
   the obstacle on a circle about its centre (radius and centre from
   MOTION_OBS_* below).  Tangent leg out at theta (the centre's direction
   + asin(R / distance)), an arc round the obstacle until heading theta
   back toward the line, then back to the line by the measured sideways
   distance (STEP_IN, ONTO, ALONG as before).  The turn-out aims at theta +
   MOTION_HD_LEAD_DEG; the leg ends at the tangent point or when within R of
   the centre; on the arc the inner wheel is trimmed by the distance to the
   centre.  Only with a valid IMU heading and the car still on the line
   (|sideways| < 30 mm); otherwise the zig-zag runs.  No look at the
   obstacle on the way. */
/* Smooth bypass (20 Nov 2026, user request: "a seamless obstacle
   avoidance"; test option, 0 = as before).  With a valid IMU heading:
     1. pivot MOTION_SM_PIVOT_DEG toward the side (the car is braked at the
        box anyway), then
     2. one continuous curve, steered every tick by pure pursuit: the
        heading wanted is toward the point MOTION_SM_LOOK_MM ahead on a path
        x(s) (s along the line from where it braked, x sideways toward the
        side):
          s 0 .. S_OUT:        out to Y on a half cosine
          S_OUT .. S_IN:       Y, alongside the box
          S_IN .. S_IN + L_IN: back to 0 on a half cosine
          beyond:              past the line at MOTION_SM_GLIDE_DEG
        so the car never stops or pivots again;
     3. once it has been out (x > Y / 2) and is past S_IN - LOOK, the line
        sensors seeing the line MOTION_LF_FOUND_TICKS ticks in a row end it:
        Line Search finds the line at once and Line Following takes over.
   Y = MOTION_SM_Y_MM + how far scan_task saw the box toward the side
   (offset - MOTION_ARC_OFFSET_CENTRED, MOTION_SM_SHIFT_MIN..MAX), so the
   car's side passes a 90 or 130 mm wide box with >= 15 mm (calculated in a
   simulation of this law on the car's geometry, line direction within 4
   deg).  The car's middle goes ~18 cm out, its outer side ~26-29 cm (sim);
   the zig-zag went 36-37 cm (log, 20:38).  The start sideways place is
   from the heading (the line sensors are on the line,
   MOTION_LINE_SENSOR_MM ahead).  Gives up (Line Search, toward the line)
   past MOTION_SM_GIVEUP_MM beyond the path or the IMU failing. */
#ifndef MOTION_BYPASS_SMOOTH
#define MOTION_BYPASS_SMOOTH      (0)
#endif
#ifndef MOTION_SM_Y_MM
#define MOTION_SM_Y_MM            (180)
#endif
#ifndef MOTION_SM_SHIFT_MIN
#define MOTION_SM_SHIFT_MIN       (-20)
#endif
#define MOTION_SM_SHIFT_MAX       (40)
#ifndef MOTION_SM_PIVOT_DEG
#define MOTION_SM_PIVOT_DEG       (35)
#endif
#ifndef MOTION_SM_S_OUT_MM
#define MOTION_SM_S_OUT_MM        (240)
#endif
#ifndef MOTION_SM_S_IN_MM
#define MOTION_SM_S_IN_MM         (600)  /* box far face <= 541 + the look  */
#endif
#ifndef MOTION_SM_L_IN_MM
#define MOTION_SM_L_IN_MM         (520)
#endif
#ifndef MOTION_LF_EDGE_LOST
#define MOTION_LF_EDGE_LOST       (0)    /* 1: off an edge, steer back at once */
#endif
#define MOTION_SM_GLIDE_DEG       (15)
#define MOTION_SM_LOOK_MM         (100)
#ifndef MOTION_SM_MAX_DEG
#define MOTION_SM_MAX_DEG         (45)
#endif
#define MOTION_SM_STEER_DEG       (3)    /* one count/tick per 3 deg of error */
#ifndef MOTION_SM_SPEED
#define MOTION_SM_SPEED           (10)   /* counts per tick, ~79 mm/s       */
#endif
#define MOTION_SM_MIN_WHEEL       (2)    /* inner wheel at least this       */
#define MOTION_SM_GIVEUP_MM       (1200)
#ifndef MOTION_SM_ONTO
#define MOTION_SM_ONTO            (1)    /* line met: ONTO + ALONG first    */
#endif
#ifndef MOTION_SM_ONTO_MM
#define MOTION_SM_ONTO_MM         (95)   /* ONTO in the smooth bypass       */
#endif
/* Curve speed (motor_task.c man_drive): 0 = off. */
#ifndef MOTION_LF_BASE_CURVE
#define MOTION_LF_BASE_CURVE      (0)
#endif
#ifndef MOTION_LF_CURVE_ERR
#define MOTION_LF_CURVE_ERR       (40)
#endif
#define MOTION_LF_CURVE_TICKS     (40u)  /* 400 ms */
#ifndef MOTION_LF_EDGE_MIN_TICKS
#define MOTION_LF_EDGE_MIN_TICKS  (15u)  /* one sensor alone 150 ms: an edge */
#endif
#ifndef MOTION_LF_STRADDLE
#define MOTION_LF_STRADDLE        (0)    /* 1: sensors either side of line */
#endif
/* Straddle steering (other team's b3_config.h values, with permission;
   curvatures 1e6 / radius mm, speeds mm/s, distances mm driven) */
#ifndef MOTION_ST_BASE_MMPS
#define MOTION_ST_BASE_MMPS       (50)
#endif
#ifndef MOTION_ST_READ_MMPS
#define MOTION_ST_READ_MMPS       (24)   /* over a barcode */
#endif
#ifndef MOTION_ST_CURVE_MMPS
#define MOTION_ST_CURVE_MMPS      (35)   /* in a learnt tight curve */
#endif
#ifndef MOTION_ST_MIN_WHEEL_MMPS
#define MOTION_ST_MIN_WHEEL_MMPS  (15)   /* inner wheel at least */
#endif
#ifndef MOTION_ST_MIN_WHEEL_PCT
#define MOTION_ST_MIN_WHEEL_PCT   (44)   /* or this share of the speed, if lower */
#endif
#ifndef MOTION_ST_KICK_CURV
#define MOTION_ST_KICK_CURV       (4400) /* nudge, 227 mm radius */
#endif
#define MOTION_ST_KICK_MIN_CURV   (1500)
#define MOTION_ST_OVERSHOOT_MM    (100)
#ifndef MOTION_ST_SHRINK_PCT
#define MOTION_ST_SHRINK_PCT      (92)
#endif
#ifndef MOTION_ST_GROW_PCT
#define MOTION_ST_GROW_PCT        (110)
#endif
#define MOTION_ST_KICK_RAMP_MM    (20)
#ifndef MOTION_ST_LEARN_CURV
#define MOTION_ST_LEARN_CURV      (300)  /* per 10 mm with a side black */
#endif
#define MOTION_ST_TIGHT_FROM_CURV (4000)
#ifndef MOTION_ST_LEARN_TIGHT_CURV
#define MOTION_ST_LEARN_TIGHT_CURV (450)
#endif
#define MOTION_ST_LEFT_PCT        (150)
#ifndef MOTION_ST_FORGET_MM
#define MOTION_ST_FORGET_MM       (80)
#endif
#define MOTION_ST_CURVE_END_MM    (40)
#define MOTION_ST_FULL_CURV       (1500) /* fully slowed: 670 mm radius */
#ifndef MOTION_ST_SLEW_CURV
#define MOTION_ST_SLEW_CURV       (400)  /* per mm */
#endif
#define MOTION_ST_ESCAPE_MM       (10)
#define MOTION_ST_BB_CROSS_MM     (30)
#ifndef MOTION_ST_LOST_MM
#define MOTION_ST_LOST_MM         (250u) /* no touch this far: lost */
#endif
#ifndef MOTION_ST_BC_HOLD
#define MOTION_ST_BC_HOLD         (0)    /* 1: hold the heading over a barcode */
#endif
#define MOTION_ST_BC_KP           (150)  /* heading hold over a barcode, curv per deg */
#define MOTION_ST_BC_MAX          (3000)
#ifndef MOTION_ST_OUTER_MM
#define MOTION_ST_OUTER_MM        (100u) /* quiet after an outside touch in a curve: out */
#endif
#ifndef MOTION_ST_EXIT_CURV
#define MOTION_ST_EXIT_CURV       (700)  /* turning (1.4 m radius or less) when a sensor loses the line */
#endif
#ifndef MOTION_ST_OUT_MAX_MM
#define MOTION_ST_OUT_MAX_MM      (120u) /* pivoting back this far (~110 deg) without the line: Line Search */
#endif
#ifndef MOTION_ST_RECOV_LOST_MM
#define MOTION_ST_RECOV_LOST_MM   (150u) /* after a recovery: no touch this far -> Line Search */
#endif
#ifndef MOTION_ST_TRAVERSE_MM
#define MOTION_ST_TRAVERSE_MM     (40u)  /* other side touched this soon: the line is sweeping across */
#endif
#ifndef MOTION_ST_FOUND_MM
#define MOTION_ST_FOUND_MM        (150u)
#endif
/* um driven for d_l + d_r (2 x counts): pi x dia / counts per turn / 2 */
#define ST_UM_2C(c2) ((int32_t)(((int64_t)(c2) * 355 * MOTION_WHEEL_DIA_MM * 1000) \
                                / ((int64_t)2 * 113 * MOTION_COUNTS_PER_REV)))
#ifndef MOTION_LF_REALIGN
#define MOTION_LF_REALIGN         (0)    /* 1: line up after a search       */
#endif
#define MOTION_RA_MIN_DEG         (20)
/* Barcode re-read (21 Nov 2026, user): a failed read (garbled or cut off)
   in Line Following -> brake, turn to the line's heading (IMU), reverse
   straight the distance read + MOTION_RT_MARGIN_MM, then Line Following at
   MOTION_LF_BASE_READ through it again.  Once per barcode: not again
   until MOTION_RT_FORGET_MM further on.  The barcode reader is gated off
   while it backs up (motion_state_read() says REVERSING). */
#ifndef MOTION_BC_RETRY
#define MOTION_BC_RETRY           (0)
#endif
#define MOTION_RT_MARGIN_MM       (80)
#define MOTION_RT_MIN_MM          (120)
#define MOTION_RT_MAX_MM          (450)
#define MOTION_RT_BACK_SPEED      (6)
#define MOTION_RT_FORGET_MM       (400)
/* Smooth bypass sized from the scan (21 Nov 2026, user: the box is turned
   either way round).  Out: half the measured face width + half the car +
   MOTION_SM_SIDE_GAP_MM (MOTION_SM_Y_MIN_MM..MAX), + the off-centre shift.
   Back in: the box's far end from the measured distance + sensor-to-axle +
   MOTION_SM_BOX_LEN_MM (its depth: the front scan cannot see it, so the
   longest side is assumed), less 15 mm (the turn starts ~12 cm before it,
   the look 10 cm ahead).  No profile: the fixed values. */
#ifndef MOTION_SM_HD_FALLBACK
#define MOTION_SM_HD_FALLBACK     (1)
#endif
#ifndef MOTION_SM_ADAPT
#define MOTION_SM_ADAPT           (0)
#endif
#ifndef MOTION_SM_BOX_LEN_MM
#define MOTION_SM_BOX_LEN_MM      (150)
#endif
#define MOTION_SM_CAR_HALF_MM     (77)
#ifndef MOTION_SM_SIDE_GAP_MM
#define MOTION_SM_SIDE_GAP_MM     (75)
#endif
#ifndef MOTION_SM_Y_MIN_MM
#define MOTION_SM_Y_MIN_MM        (185)
#endif
#ifndef MOTION_SM_Y_MAX_MM
#define MOTION_SM_Y_MAX_MM        (240)
#endif
#ifndef MOTION_SM_TURN_EARLY_MM
#define MOTION_SM_TURN_EARLY_MM   (0)    /* turn this far before the far end */
#endif
#ifndef MOTION_SM_BOX_DIR
#define MOTION_SM_BOX_DIR         (0)    /* 1: the bypass takes the line's direction from the box */
#endif
#ifndef MOTION_SM_BOX_DIR_MAX
#define MOTION_SM_BOX_DIR_MAX     (30)
#endif
#ifndef MOTION_SM_MISS_X_MM
#define MOTION_SM_MISS_X_MM       (250)  /* past where the line should be, no line: search */
#endif
#ifndef MOTION_SM_CROSS_CHECK_MM
#define MOTION_SM_CROSS_CHECK_MM  (0)    /* >0: a touch is taken after this mm, unless both black (a crossing line) */
#endif
#ifndef MOTION_SM_CROSS_DEG
#define MOTION_SM_CROSS_DEG       (40)
#endif
#ifndef MOTION_SM_SEEN_DEG
#define MOTION_SM_SEEN_DEG        (8)    /* line counts only heading in this much */
#endif
#ifndef MOTION_SM_SEEN_X_MM
#define MOTION_SM_SEEN_X_MM       (90)   /* smooth: line counts only this near */
#endif
#ifndef MOTION_LF_GAP_MM
#define MOTION_LF_GAP_MM          (0)    /* white: hold heading this far   */
#endif
#define MOTION_LF_GAP_K           (25)   /* x SCALE per degree off          */
#ifndef MOTION_LF_CURVE_RUN
#define MOTION_LF_CURVE_RUN       (12u)  /* 120 ms at one edge: a curve     */
#endif
#ifndef MOTION_LF_CURVE_BOOST_X10
#define MOTION_LF_CURVE_BOOST_X10 (16)   /* correction x1.6 at curve speed  */
#endif
#ifndef MOTION_SM_SETTLE_TICKS
#define MOTION_SM_SETTLE_TICKS    (20u)  /* 200 ms braked after ALONG       */
#endif
#ifndef MOTION_LF_RATE_DAMP
#define MOTION_LF_RATE_DAMP       (0)    /* x SCALE per count/tick of turn  */
#endif
/* Side watch (20 Nov 2026, user request): during the smooth bypass the
   ultrasonic turns 45 deg toward the box (scan_task SCAN_FLG_WATCH) and
   pings every 60 ms.  Once it has seen the box (a reading under
   MOTION_SM_WATCH_NEAR_MM) and then reads clear twice, the way back in
   starts: at 45 deg the beam meets the box's side ~10 cm ahead of the nose,
   so the turn starts MOTION_SM_CLEAR_MM later (path S_IN = that + the
   look), never before MOTION_SM_S_IN_MIN_MM, at the latest
   MOTION_SM_S_IN_MAX_MM.  Never seen by MOTION_SM_S_IN_MM - look: the
   distance rule (MOTION_SM_S_IN_MM).  0 = distance rule only. */
#ifndef MOTION_SM_WATCH
#define MOTION_SM_WATCH           (0)
#endif
#define MOTION_SM_WATCH_NEAR_MM   (250)
#ifndef MOTION_SM_CLEAR_MM
#define MOTION_SM_CLEAR_MM        (150)
#endif
/* 90 deg side watch (MOTION_SM_WATCH 2, 21 Nov 2026): the sensor square on
   to the box's side while the car runs alongside it (heading within
   MOTION_SM_WATCH_PAR_DEG, out within 40 mm of the pass).  A reading
   closer than the line is the box; once seen, two readings at or beyond
   the line mean the nose has passed its far end (the beam's footprint
   reaches ~35 mm past it at 12-15 cm): the box ends ~120 mm ahead of the
   axle, and the way back starts there (S_IN = s + MOTION_SM_CLEAR_MM +
   the look, CLEAR = 120).  Never seen: the far end of a
   MOTION_SM_BOX_LEN_MM deep box from the scan's distance. */
#define MOTION_SM_WATCH_PAR_DEG   (15)
#ifndef MOTION_SM_WATCH_DEG
#define MOTION_SM_WATCH_DEG       (90)   /* = scan_task's SCAN_WATCH_DEG */
#endif
/* From the side echoes (21 Nov 2026, after Buddy 5's s5_avoid of the other
   team, used with permission): widen the pass when the box's side is
   nearer than planned, place the far end from the echoes, and start the
   way back as early as the side gap allows. */
#define MOTION_SM_WIDEN_GAP_MM    (40)   /* car side to the box side, wanted */
#define MOTION_SM_WIDEN_MAX_MM    (60)   /* out by at most this              */
#define MOTION_SM_FAR_MARGIN_MM   (20)   /* beyond the furthest echo         */
#ifndef MOTION_SM_CLEAR_MIN_MM
#define MOTION_SM_CLEAR_MIN_MM    (215)  /* nose 155 + the 70 deg ray's reach */
#endif
#define MOTION_SM_EARLY_GAP_MM    (50)   /* no early turn below this gap     */
#define MOTION_SM_EARLY_MAX_MM    (30)   /* turn at most this before the end */
#ifndef MOTION_SM_S_IN_MIN_MM
#define MOTION_SM_S_IN_MIN_MM     (480)
#endif
#define MOTION_SM_S_IN_MAX_MM     (850)

#ifndef MOTION_BYPASS_ARC
#define MOTION_BYPASS_ARC         (0)
#endif
/* The obstacle the arc goes round (the sensor cannot tell its shape or
   length, so they are set here; the demo box by default):
     MOTION_OBS_ROUND 0: a box W x L (a cube: W = L), reach from its centre
       = half the diagonal, sqrt((W/2)^2 + (L/2)^2): 79 mm for 90 x 130;
     MOTION_OBS_ROUND 1: a cylinder of diameter W, reach W / 2.
   Circle radius R = reach + half the car (63) + MOTION_ARC_GAP_MM.
   Centre ahead of the axle where it braked: C = MOTION_ARC_STOP_MM + 155
   (sensor to axle) + half its length along the line (L / 2, or W / 2 for a
   cylinder).  Demo box: R = 79 + 63 + 100 = 242, C = 240 + 155 + 65 = 460,
   theta 32 deg; a 200 mm cube: R 304, C 495, theta 38 deg; a 100 mm
   cylinder: R 213, C 445, theta 29 deg (calculated).  The centre's
   sideways place comes from scan_task's offset (box off the line). */
#define MOTION_SENSOR_AXLE_MM     (155)  /* axle to the car front / sensor, measured 1 Nov */
#ifndef MOTION_OBS_W_MM
#define MOTION_OBS_W_MM           (90)
#endif
#ifndef MOTION_OBS_L_MM
#define MOTION_OBS_L_MM           (130)
#endif
#ifndef MOTION_OBS_ROUND
#define MOTION_OBS_ROUND          (0)
#endif
#ifndef MOTION_ARC_GAP_MM
#define MOTION_ARC_GAP_MM         (100)  /* car side to the obstacle's reach */
#endif
#define MOTION_ARC_STOP_MM        (240)  /* sensor to its face at the stop (223-252 seen) */
#define MOTION_ARC_OFFSET_CENTRED (293)  /* scan_task: 45 + C_pass 248, centred */
#define MOTION_ARC_MAX_SHIFT_MM   (150)  /* centre off the line: at most this */

/* Measured on the car, docs/motion/results.md: counts per wheel turn
   (x4 quadrature, 2 trials), wheel diameter and the distance between the
   two wheels (ruler).  Distance uses the first two; turn rate uses all
   three.  Calibrate with the NFR-04 straight trials. */
#define MOTION_COUNTS_PER_REV (2535)
#define MOTION_WHEEL_DIA_MM   (64)

/* The right wheel's own counts per turn (x4).  2535, the same as the left,
   so the scaling is off.  Measured 19 Nov 2026 (M, bench image, 10 hand
   turns each, ~/pico_session/logs/serial_20261119_222026_S1.log): right
   20234 (444 missed transitions), then 23558 (125 missed), then after the
   right encoder's Grove cable was unplugged and re-seated 25315 (13
   missed) = 2532 per turn; left 25330 (0 missed) = 2533.  The 4 Nov value
   1590 (Buddy 5: hand 1582-1598, open loop R 3101-3111 vs L 5018-5049)
   came from the same fault: a loose right encoder cable loses counts and
   reads as missed transitions; there is no different gearbox.  23-24 Oct
   gave 2533 and 2536.8 (results.md).  If "missed" climbs on the right
   again, re-seat that cable before trusting any distance or turn.
   motion_task still scales right counts to left-wheel units on read
   (tick_encoders) whenever this differs from MOTION_COUNTS_PER_REV.  See
   docs/motion/right_encoder_scaling.md. */
#define MOTION_COUNTS_PER_REV_R (2535)
#define MOTION_WHEELBASE_MM   (127)

/* Hump (W6 6.2 / 6.4, Integration Rules 2.7; M3), while imu_task's
   hump_state is not NONE (imu_word_read(), one aligned load, D1):
   - every wheel speed target is capped at V_hump = 15 cm/s, in counts per
     tick: v x counts per turn / (pi x wheel diameter x 100 ticks/s), with
     pi as 355/113, rounded down (18 at 150 mm/s; calculated);
   - the line may be missing 2 x t_lost (LINE_LOST_MS) before Line Search.
     line_state's dark time saturates at 2550 ms (8 bits x 10 ms), so past
     that motion counts its own ticks: MOTION_HUMP_LOST_TICKS more ticks at
     the saturated value = 2 x 1500 ms. */
#define MOTION_HUMP_MAX_MMPS   (150)
#define MOTION_MMPS_TO_CPT(v)                                                  \
    (((v) * MOTION_COUNTS_PER_REV * 113)                                       \
     / (355 * MOTION_WHEEL_DIA_MM * 100))
#define MOTION_HUMP_MAX_CPT    (MOTION_MMPS_TO_CPT(MOTION_HUMP_MAX_MMPS))
#define MOTION_HUMP_LOST_MS    (2 * LINE_LOST_MS)

/* Turn rate and acceleration are averaged over this many ticks.
   PROPOSED: 5 ticks = 50 ms, the most agreed with Buddy 4 (23 Oct), so
   her 5-sample stationary rule still works.  One count of difference
   between the wheels is then about 0.72 deg/s (calculated). */
#define MOTION_DS_WINDOW_TICKS (5u)

/* Forward acceleration at or above this reads ACCELERATING (and at or
   below minus this, DECELERATING), so imu_task holds its pitch.
   PROPOSED: 171 mm/s^2 tilts an accelerometer's pitch reading by 1 degree
   (9807 x tan 1 deg, calculated).  Set from Buddy 4's A4 hump tests. */
#define MOTION_DS_ACCEL_MMPS2 (171)

/* ------------------------------------------------------------------ *
 *  Bench test  (wheels off the ground!)
 * ------------------------------------------------------------------ */

/* Used only when MOTION_BENCH is 1.
   0: open-loop sequence (start here).
   1: closed-loop speed test, only after the open-loop test passes and
      the four INVERT flags above are right. */
#ifndef MOTION_TEST_PID
#define MOTION_TEST_PID (0)
#endif

/* Used only when MOTION_BENCH is 1.
   1: watchdog test.  The open-loop test stops feeding the watchdog
      MOTION_TEST_WDT_TICKS into "L fwd", with the left wheel turning,
      and spins.  The chip must reset MOTION_WDT_MS later with the wheel
      stopped; the next boot prints how long after the last feed. */
#ifndef MOTION_TEST_WDT
#define MOTION_TEST_WDT (0)
#endif
#define MOTION_TEST_WDT_TICKS (50u) /* 0.5 s */

/* Used only when MOTION_BENCH is 1.
   1: stop test (NFR-18).  The button starts both wheels at
      MOTION_TEST_DUTY; a stand-in for scan_task and imu_task (a task on
      core 1 at imu_task's priority) sets STOP_SCAN and STOP_IMU in turn
      while they turn, and clears each again (scan_task stays print-only
      in bench images: see motor_task_flag()).  Every stop prints how long
      after the request the brake was written; after
      MOTION_TEST_STOP_TRIALS stops per source the summary prints. */
#ifndef MOTION_TEST_STOP
#define MOTION_TEST_STOP (0)
#endif
#define MOTION_TEST_STOP_TRIALS (100u) /* per source                  */
#define MOTION_TEST_STOP_DRIVE  (30u)  /* ticks driving before a stop */
#define MOTION_TEST_STOP_HOLD   (20u)  /* stand-in steps, about 400 ms */
#define MOTION_TEST_STOP_PRI    (12)   /* imu_task's priority         */
/* Each stand-in step is about 20 ms, as tk_dly_tsk(10) adds one 10 ms
   kernel tick, so a STOP stays set about 400 ms (C). */

#define MOTION_TEST_DUTY       (350)  /* open-loop steps, per-mille */
#define MOTION_TEST_STEP_TICKS (150u) /* 1.5 s per step */
#define MOTION_TEST_GAP_TICKS  (100u) /* 1 s braked between steps */

/* Dead-band ramp: both wheels from 0 upward in small steps. */
#define MOTION_RAMP_STEP_DUTY   (25)
#define MOTION_RAMP_STEP_TICKS  (30u) /* 300 ms per level */
#define MOTION_RAMP_MAX_DUTY    (600)
#define MOTION_RAMP_MOVE_COUNTS (10) /* counts per level = "moving" */

/* Closed-loop test.  Target in counts per 10 ms tick: set it to about
   half of what the open-loop "both fwd" step printed per tick. */
#define MOTION_PID_TARGET    (20)
#define MOTION_PID_TICKS     (300u) /* 3 s */
#define MOTION_PID_KP_Q8     (2048) /* 8 per-mille per count/tick */
#define MOTION_PID_KI_Q8     (256)  /* 1 per-mille per count/tick/tick */
#define MOTION_PID_KD_Q8     (0)
#define MOTION_PID_OUT_LIMIT (600) /* bench cap, per-mille */

#endif /* MOTION_CFG_H */
