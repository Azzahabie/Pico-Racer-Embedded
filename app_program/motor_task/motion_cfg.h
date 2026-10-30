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
   Both wheels run under the PID at MOTION_LF_BASE; the line error
   (-100..+100, + = turn left) moves the two targets apart by up to
   MOTION_LF_GAIN counts per tick each way.  PROPOSED: 15 counts per tick
   is about 12 cm/s (calculated); tune both on the floor. */
#define MOTION_LF_BASE        (15)
#define MOTION_LF_GAIN        (8)
#define MOTION_LF_SEEN_MS     (30u) /* dark below this: line still seen */
#define MOTION_LF_FOUND_TICKS (3u)  /* seen this many ticks: found      */

/* Line search (statechart LINE_SEARCH, [L_search]).  Both sensors sit
   inside the 18 mm line, so a centred car reads 11 and only a car off the
   line reads 00: after LINE_LOST_MS of 00 the car pivots on the spot
   toward the side the line was last seen (|error| >= MOTION_LS_DIR_MIN;
   the sign already includes LINE_STEER_INVERT), up to MOTION_LS_SWEEP_DEG,
   then back past the start to the same angle the other side.  The line
   under a sensor ends it (LINE_FOUND); a full sweep without it is
   LINE_LOST.  PROPOSED: 6 counts per tick per wheel, about 5 cm/s. */
#define MOTION_LS_SPEED     (6)
#define MOTION_LS_SWEEP_DEG (45)
#define MOTION_LS_DIR_MIN   (10)

/* ------------------------------------------------------------------ *
 *  Car geometry and distance_state  (motion_ds.h)
 * ------------------------------------------------------------------ */

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
