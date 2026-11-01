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

/* No tick for this long means the cyclic handler or the kernel timer has
   stopped: brake. */
#define MOTION_TICK_TIMEOUT_MS (50)

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
   2.5 counts, and the soft cap leaves the inner wheel at 1 count. */
#define MOTION_LF_BASE        (4)
#define MOTION_LF_SCALE       (100)
#define MOTION_LF_GAIN        (5)   /* e 60 and up -> the soft cap       */
#define MOTION_LF_DAMP        (15)  /* per unit of e change              */
#define MOTION_LF_DAMP_TICKS  (5u)  /* change measured over 50 ms        */
#define MOTION_LF_SOFT_MAX    ((MOTION_LF_BASE - 1) * MOTION_LF_SCALE)
#define MOTION_LF_SLEW        (50)  /* 0 to SOFT_MAX in 60 ms            */
#define MOTION_LF_HARD_ERR    (80)
#define MOTION_LF_HARD_TICKS  (25u) /* 250 ms                            */
#define MOTION_LF_HARD_MAX    ((MOTION_LF_BASE + 1) * MOTION_LF_SCALE)
#define MOTION_LF_SEEN_MS     (30u) /* dark below this: line still seen */
#define MOTION_LF_FOUND_TICKS (3u)  /* seen this many ticks: found      */

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
#define MOTION_LS_SWEEP_DEG (45)
#define MOTION_LS_DIR_MIN   (10)

/* ------------------------------------------------------------------ *
 *  Car geometry and distance_state  (motion_ds.h)
 * ------------------------------------------------------------------ */

/* Turning (statechart TURNING, FR-08).  nav sends JUNCTION once GP7 has
   been on the crossbar LINE_JUNCTION_SAMPLES; the car then drives
   MOTION_TURN_FWD_MM straight so the axle is over the crossing ([d_sensor]:
   GP7 to the wheel axle), and pivots on the spot at MOTION_TURN_SPEED
   until each wheel has turned PIVOT_COUNTS(90) (left, right) or (180)
   (U-turn, counter-clockwise).  Straight on drives MOTION_TURN_CROSS_MM
   ([L_cross]) across the crossing, ignoring the line.  Then MOVE_DONE, and
   Line Following if the line is under the sensors, else Line Search.
   PROPOSED: measure d_sensor on the car; turn speed as the line search. */
#define MOTION_TURN_FWD_MM    (50)
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
#define MOTION_BYPASS_SPEED       (12)
#define MOTION_BYPASS_TURN_SPEED  (10)

/* Measured on the car, docs/motion/results.md: counts per wheel turn
   (x4 quadrature, 2 trials), wheel diameter and the distance between the
   two wheels (ruler).  Distance uses the first two; turn rate uses all
   three.  Calibrate with the NFR-04 straight trials. */
#define MOTION_COUNTS_PER_REV (2535)
#define MOTION_WHEEL_DIA_MM   (64)
#define MOTION_WHEELBASE_MM   (127)

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
