/*
 *	test_motion_lib.c
 *	Host unit tests for app_program/motor_task/motion_lib.c (Buddy 2).
 *
 *	Runs on a PC with plain gcc; no kernel, no hardware:
 *	    make -C tests/motion_lib
 *	Built with -fsanitize=undefined so a signed overflow fails the run.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "motion_lib.h"

static int g_checks;
static int g_failures;

#define CHECK(cond)							\
	do {								\
		g_checks++;						\
		if (!(cond)) {						\
			g_failures++;					\
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		}							\
	} while (0)

#define CHECK_EQ(actual, expected)					\
	do {								\
		long long a_ = (long long)(actual);			\
		long long e_ = (long long)(expected);			\
		g_checks++;						\
		if (a_ != e_) {						\
			g_failures++;					\
			printf("FAIL %s:%d: %s == %lld, expected %lld\n",	\
			       __FILE__, __LINE__, #actual, a_, e_);	\
		}							\
	} while (0)

/* Forward order of (A << 1) | B states: A leads B. */
static const uint8_t FWD_SEQ[4] = { 0u, 2u, 3u, 1u };

/* ------------------------------------------------------------------ */

static void test_quad_table_properties(void)
{
	motion_quad_t q;
	uint8_t prev, curr;

	for (prev = 0u; prev < 4u; prev++) {
		for (curr = 0u; curr < 4u; curr++) {
			uint8_t changed = (uint8_t)(prev ^ curr);
			motion_quad_result_t fwd, back;

			motion_quad_init(&q, prev);
			fwd = motion_quad_step(&q, curr);
			motion_quad_init(&q, curr);
			back = motion_quad_step(&q, prev);

			if (changed == 0u) {
				CHECK_EQ(fwd, MOTION_QUAD_NONE);
			} else if (changed == 3u) {
				CHECK_EQ(fwd, MOTION_QUAD_INVALID);
				CHECK_EQ(back, MOTION_QUAD_INVALID);
			} else {
				/* One channel changed: a real step, and the
				   reverse transition is the opposite step. */
				CHECK(fwd == MOTION_QUAD_FWD || fwd == MOTION_QUAD_REV);
				CHECK((fwd == MOTION_QUAD_FWD && back == MOTION_QUAD_REV) ||
				      (fwd == MOTION_QUAD_REV && back == MOTION_QUAD_FWD));
			}
		}
	}
}

static void test_quad_counting(void)
{
	motion_quad_t q;
	int i;

	/* 100 full cycles forward = 400 counts. */
	motion_quad_init(&q, FWD_SEQ[0]);
	for (i = 1; i <= 400; i++) {
		CHECK_EQ(motion_quad_step(&q, FWD_SEQ[i % 4]), MOTION_QUAD_FWD);
	}
	CHECK_EQ(q.count, 400);
	CHECK_EQ(q.invalid, 0);

	/* Back to zero, then one more step backward wraps to 65535. */
	for (i = 399; i >= 0; i--) {
		CHECK_EQ(motion_quad_step(&q, FWD_SEQ[i % 4]), MOTION_QUAD_REV);
	}
	CHECK_EQ(q.count, 0);
	CHECK_EQ(motion_quad_step(&q, FWD_SEQ[3]), MOTION_QUAD_REV);
	CHECK_EQ(q.count, 65535);

	/* Repeated identical states do nothing. */
	CHECK_EQ(motion_quad_step(&q, FWD_SEQ[3]), MOTION_QUAD_NONE);
	CHECK_EQ(q.count, 65535);
}

static void test_quad_invalid(void)
{
	motion_quad_t q;

	motion_quad_init(&q, 0u);
	CHECK_EQ(motion_quad_step(&q, 3u), MOTION_QUAD_INVALID);
	CHECK_EQ(q.count, 0);
	CHECK_EQ(q.invalid, 1);
	/* Re-synchronised: from 11 the forward step is 11 -> 01. */
	CHECK_EQ(motion_quad_step(&q, 1u), MOTION_QUAD_FWD);
	CHECK_EQ(q.count, 1);

	/* The invalid counter saturates instead of wrapping. */
	q.invalid = 65534u;
	q.state = 0u;
	(void)motion_quad_step(&q, 3u);
	(void)motion_quad_step(&q, 0u);
	CHECK_EQ(q.invalid, 65535);
	q.state = 0u;
	(void)motion_quad_step(&q, 3u);
	CHECK_EQ(q.invalid, 65535);
}

static void test_quad_ab_swap(void)
{
	motion_quad_t q;
	int i;

	CHECK_EQ(motion_quad_ab(1u, 0u, false), 2u);
	CHECK_EQ(motion_quad_ab(0u, 1u, false), 1u);
	CHECK_EQ(motion_quad_ab(1u, 0u, true), 1u);
	CHECK_EQ(motion_quad_ab(0x400u, 0x8u, false), 3u);	/* any non-zero level */

	/* Same physical pin sequence counts the other way when swapped. */
	motion_quad_init(&q, motion_quad_ab(0u, 0u, true));
	for (i = 1; i <= 8; i++) {
		uint8_t s = FWD_SEQ[i % 4];
		uint32_t a = (s >> 1) & 1u, b = s & 1u;
		CHECK_EQ(motion_quad_step(&q, motion_quad_ab(a, b, true)),
			 MOTION_QUAD_REV);
	}
	CHECK_EQ(q.count, 65528);	/* -8 */
}

/* ------------------------------------------------------------------ */

static void test_pack(void)
{
	uint32_t i;

	CHECK_EQ(motion_pack_counts(0x1234u, 0xABCDu), 0xABCD1234u);
	CHECK_EQ(motion_unpack_left(0xABCD1234u), 0x1234u);
	CHECK_EQ(motion_unpack_right(0xABCD1234u), 0xABCDu);

	srand(1u);
	for (i = 0u; i < 10000u; i++) {
		uint16_t l = (uint16_t)rand(), r = (uint16_t)rand();
		uint32_t p = motion_pack_counts(l, r);
		CHECK_EQ(motion_unpack_left(p), l);
		CHECK_EQ(motion_unpack_right(p), r);
	}
}

static void test_delta(void)
{
	CHECK_EQ(motion_count_delta(10u, 10u), 0);
	CHECK_EQ(motion_count_delta(15u, 10u), 5);
	CHECK_EQ(motion_count_delta(10u, 15u), -5);
	CHECK_EQ(motion_count_delta(2u, 65534u), 4);		/* forward wrap */
	CHECK_EQ(motion_count_delta(65534u, 2u), -4);		/* backward wrap */
	CHECK_EQ(motion_count_delta(32767u, 0u), 32767);
	CHECK_EQ(motion_count_delta(32768u, 0u), -32768);
	CHECK_EQ(motion_count_delta(0u, 32768u), -32768);
}

/* Random walk: the summed deltas must equal the true 64-bit position. */
static void test_delta_random_walk(void)
{
	long long truth = 0, summed = 0;
	uint16_t prev = 0u;
	int tick;

	srand(7u);
	for (tick = 0; tick < 200000; tick++) {
		int move = (rand() % 6001) - 3000;	/* |move| < 32768 */
		uint16_t now;

		truth += move;
		now = (uint16_t)((uint64_t)truth & 0xFFFFu);
		summed += motion_count_delta(now, prev);
		prev = now;
	}
	CHECK_EQ(summed, truth);
}

/* ------------------------------------------------------------------ */

#define S_NONE	MOTION_STALL_NONE
#define S_MOTOR	MOTION_STALL_MOTOR
#define S_ENC	MOTION_STALL_ENCODER

/* Tick of the first non-NONE result (or 0) and the result itself. */
static int stall_first(motion_stall_t *p, bool b_driven, int32_t dir,
		       int ticks, int32_t delta, uint32_t ea, uint32_t eb,
		       motion_stall_result_t *p_res)
{
	int i;

	*p_res = S_NONE;
	for (i = 1; i <= ticks; i++) {
		*p_res = motion_stall_update(p, b_driven, dir, delta, ea, eb);
		if (*p_res != S_NONE) {
			return i;
		}
	}
	return 0;
}

static void test_stall_basics(void)
{
	motion_stall_t s;
	motion_stall_result_t r;
	int i;

	motion_stall_init(&s, 30u, 3, 3u);

	/* Not driven: never a stall, however long. */
	CHECK_EQ(stall_first(&s, false, 1, 1000, 0, 0u, 0u, &r), 0);

	/* Driven with no movement and no edges: motor stall on tick 30. */
	motion_stall_reset(&s);
	CHECK_EQ(stall_first(&s, true, 1, 40, 0, 0u, 0u, &r), 30);
	CHECK(r == S_MOTOR);

	/* Progress starts a fresh window: net and both edge counts clear. */
	motion_stall_reset(&s);
	CHECK(motion_stall_update(&s, true, 1, 3, 3u, 0u) == S_NONE);
	CHECK_EQ(s.net, 0);
	CHECK_EQ(s.edges_a, 0);
	CHECK_EQ(s.edges_b, 0);
	CHECK_EQ(s.idle_ticks, 0);

	/* Moving well, either mode: never a stall. */
	motion_stall_reset(&s);
	CHECK_EQ(stall_first(&s, true, 1, 1000, 5, 3u, 2u, &r), 0);
	motion_stall_reset(&s);
	CHECK_EQ(stall_first(&s, true, 0, 1000, -5, 3u, 2u, &r), 0);

	/* One count per 10 ticks reaches 3 counts by tick 30: no stall. */
	motion_stall_reset(&s);
	for (i = 1; i <= 300; i++) {
		bool b_step = (i % 10 == 0);
		CHECK(motion_stall_update(&s, true, 1, b_step ? 1 : 0,
					  (b_step && (i % 20 == 0)) ? 1u : 0u,
					  (b_step && (i % 20 != 0)) ? 1u : 0u) == S_NONE);
	}

	/* One count per 15 ticks: only 2 counts by tick 30, a motor stall
	   (two edges, one per phase: too few to call it an encoder fault). */
	motion_stall_reset(&s);
	for (i = 1, r = S_NONE; i <= 45 && r == S_NONE; i++) {
		bool b_step = (i % 15 == 0);
		r = motion_stall_update(&s, true, 1, b_step ? 1 : 0,
					(b_step && i == 15) ? 1u : 0u,
					(b_step && i == 30) ? 1u : 0u);
	}
	CHECK_EQ(i - 1, 30);
	CHECK(r == S_MOTOR);

	/* Closed loop: moving the wrong way is not progress (a wrong INVERT
	   flag ends as a motor stall, not a runaway). */
	motion_stall_reset(&s);
	CHECK_EQ(stall_first(&s, true, 1, 40, -50, 25u, 25u, &r), 30);
	CHECK(r == S_MOTOR);
	/* Open loop: either way counts. */
	motion_stall_reset(&s);
	CHECK_EQ(stall_first(&s, true, 0, 1000, -50, 25u, 25u, &r), 0);

	/* Stopping the drive clears the window. */
	motion_stall_reset(&s);
	CHECK_EQ(stall_first(&s, true, 1, 29, 0, 0u, 0u, &r), 0);
	CHECK(motion_stall_update(&s, false, 1, 0, 0u, 0u) == S_NONE);
	CHECK(motion_stall_update(&s, true, 1, 0, 0u, 0u) == S_NONE);

	/* A new commanded direction restarts the window: 20 ticks forward,
	   then reverse, stalls 30 ticks after the change, not 10. */
	motion_stall_reset(&s);
	CHECK_EQ(stall_first(&s, true, 1, 20, 0, 0u, 0u, &r), 0);
	CHECK_EQ(stall_first(&s, true, -1, 40, 0, 0u, 0u, &r), 30);
	CHECK(r == S_MOTOR);

	/* Edges on one phase but fewer than enc_edges: still a motor stall. */
	motion_stall_reset(&s);
	CHECK(motion_stall_update(&s, true, 1, 0, 2u, 0u) == S_NONE);
	CHECK_EQ(stall_first(&s, true, 1, 40, 0, 0u, 0u, &r), 29);
	CHECK(r == S_MOTOR);

	/* The encoder-fault threshold is its own setting, not min_counts:
	   with enc_edges 8, 5 edges on one phase is a motor stall, 8 is an
	   encoder fault. */
	motion_stall_init(&s, 30u, 3, 8u);
	CHECK(motion_stall_update(&s, true, 1, 0, 5u, 0u) == S_NONE);
	CHECK_EQ(stall_first(&s, true, 1, 40, 0, 0u, 0u, &r), 29);
	CHECK(r == S_MOTOR);
	motion_stall_reset(&s);
	CHECK(motion_stall_update(&s, true, 1, 0, 8u, 0u) == S_NONE);
	CHECK_EQ(stall_first(&s, true, 1, 40, 0, 0u, 0u, &r), 29);
	CHECK(r == S_ENC);

	/* Each phase counts only since the other last moved: a stray edge
	   on the quiet phase early on does not hide a dead wire later. */
	motion_stall_init(&s, 30u, 3, 3u);
	CHECK(motion_stall_update(&s, true, 1, 0, 0u, 1u) == S_NONE); /* B once */
	CHECK_EQ(stall_first(&s, true, 1, 40, 0, 1u, 0u, &r), 29);    /* then A */
	CHECK(r == S_ENC);

	/* min_counts below 2 is raised to 2 (a single dithering count is
	   never enough progress); enc_edges 0 is raised to 1. */
	motion_stall_init(&s, 30u, 1, 0u);
	CHECK_EQ(s.min_counts, MOTION_STALL_MIN_COUNTS_LOW);
	CHECK_EQ(s.enc_edges, 1);
}

/* Pins of an encoder at position c (forward 00 10 11 01, A leads), with
   the B wire unplugged (held high by the pull-up) when b_dead_b. */
static uint8_t quad_pins(long c, bool b_dead_b)
{
	unsigned m = (unsigned)(c & 3L);
	uint32_t a = ((m == 1u) || (m == 2u)) ? 1u : 0u;
	uint32_t b = ((m == 2u) || (m == 3u)) ? 1u : 0u;

	if (b_dead_b) {
		b = 1u;
	}
	return motion_quad_ab(a, b, false);
}

/* The B wire comes off while the wheel is turning, at several speeds and
   moments: the window then already holds edges from before the failure,
   and the pull-up makes one more B edge when B was low.  It must still
   be an encoder fault, within one stall window of the failure. */
static void test_stall_wire_comes_off(void)
{
	int speed, fail_tick;

	for (speed = 1; speed <= 20; speed++) {
		for (fail_tick = 10; fail_tick < 18; fail_tick++) {
			motion_quad_t q;
			motion_stall_t s;
			motion_stall_result_t r = S_NONE;
			bool b_dead = false;
			long pos = 0;
			uint8_t ab = quad_pins(0, false);
			uint16_t prev;
			int tick, k;

			motion_quad_init(&q, ab);
			motion_stall_init(&s, 30u, 3, 3u);
			prev = q.count;
			for (tick = 1; tick <= 200 && r == S_NONE; tick++) {
				uint32_t ea = 0u, eb = 0u;
				uint8_t now;

				if (tick == fail_tick) {
					b_dead = true;		/* wire pulled */
					now = quad_pins(pos, true);
					if (now != ab) {
						eb++;		/* the pull-up's edge */
						(void)motion_quad_step(&q, now);
						ab = now;
					}
				}
				for (k = 0; k < speed; k++) {
					pos++;
					now = quad_pins(pos, b_dead);
					if (now != ab) {
						if (((now ^ ab) & 2u) != 0u) ea++;
						if (((now ^ ab) & 1u) != 0u) eb++;
						(void)motion_quad_step(&q, now);
						ab = now;
					}
				}
				r = motion_stall_update(&s, true, 1,
							motion_count_delta(q.count, prev), ea, eb);
				prev = q.count;
			}
			CHECK(r == S_ENC);
			CHECK((tick - 1) - fail_tick <= 32);
		}
	}
}

/* A healthy run first (the window restarts many times), then a dead
   wire on the same detector, without re-initialising it. */
static void test_stall_healthy_then_dead(void)
{
	motion_stall_t s;
	motion_stall_result_t r;
	int i;

	motion_stall_init(&s, 30u, 3, 3u);
	for (i = 0; i < 100; i++) {
		CHECK(motion_stall_update(&s, true, 1, 4, 2u, 2u) == S_NONE);
	}
	/* Dead B: A toggles twice a tick, the count rocks +1/-1. */
	for (i = 1, r = S_NONE; i <= 40 && r == S_NONE; i++) {
		r = motion_stall_update(&s, true, 1, (i % 2 == 1) ? 1 : -1, 2u, 0u);
	}
	CHECK_EQ(i - 1, 30);
	CHECK(r == S_ENC);
}

/* One encoder wire unplugged: the pull-up holds that phase at a fixed
   level while the other phase keeps toggling.  Run it through the real
   decoder at many speeds, both modes, every stuck pattern: it must be an
   encoder fault on exactly the 30th tick, never 'moving'. */
static void test_stall_dead_phase(void)
{
	int stuck, edges_per_tick, mode;

	for (stuck = 0; stuck < 4; stuck++) {	/* A=0, A=1, B=0, B=1 */
		for (mode = 0; mode < 2; mode++) {
			for (edges_per_tick = 1; edges_per_tick <= 40; edges_per_tick++) {
				motion_quad_t q;
				motion_stall_t s;
				motion_stall_result_t r = S_NONE;
				bool b_dead_a = (stuck < 2);
				uint8_t level = (uint8_t)(stuck & 1);
				uint8_t live = 0u;
				uint16_t prev;
				int tick, e;

				motion_quad_init(&q, b_dead_a ? motion_quad_ab(level, live, false)
							      : motion_quad_ab(live, level, false));
				motion_stall_init(&s, 30u, 3, 3u);
				prev = q.count;
				for (tick = 1; tick <= 100 && r == S_NONE; tick++) {
					for (e = 0; e < edges_per_tick; e++) {
						live ^= 1u;
						(void)motion_quad_step(&q, b_dead_a
							? motion_quad_ab(level, live, false)
							: motion_quad_ab(live, level, false));
					}
					r = motion_stall_update(&s, true, (mode == 0) ? 0 : 1,
								motion_count_delta(q.count, prev),
								b_dead_a ? 0u : (uint32_t)edges_per_tick,
								b_dead_a ? (uint32_t)edges_per_tick : 0u);
					prev = q.count;
				}
				CHECK_EQ(tick - 1, 30);
				CHECK(r == S_ENC);
				CHECK_EQ(q.invalid, 0);
			}
		}
	}
}

/* ------------------------------------------------------------------ */

static void test_pid_basic(void)
{
	motion_pid_t pid;

	/* P only, Kp = 2.0: output = 2 * error. */
	motion_pid_init(&pid, 512, 0, 0, 1000);
	CHECK_EQ(motion_pid_step(&pid, 100, 40), 120);
	CHECK_EQ(motion_pid_step(&pid, 40, 100), -120);
	CHECK_EQ(motion_pid_step(&pid, 0, 0), 0);

	/* Saturation. */
	CHECK_EQ(motion_pid_step(&pid, 4000, 0), 1000);
	CHECK_EQ(motion_pid_step(&pid, -4000, 0), -1000);

	/* I only, Ki = 1.0: output ramps by the error each step. */
	motion_pid_init(&pid, 0, 256, 0, 1000);
	CHECK_EQ(motion_pid_step(&pid, 10, 0), 10);
	CHECK_EQ(motion_pid_step(&pid, 10, 0), 20);
	CHECK_EQ(motion_pid_step(&pid, 10, 0), 30);
	motion_pid_reset(&pid);
	CHECK_EQ(motion_pid_step(&pid, 10, 0), 10);

	/* D only, Kd = 1.0: no kick on the first step, then the change. */
	motion_pid_init(&pid, 0, 0, 256, 1000);
	CHECK_EQ(motion_pid_step(&pid, 50, 0), 0);
	CHECK_EQ(motion_pid_step(&pid, 80, 0), 30);

	/* Out-of-range settings are clamped, not trusted. */
	motion_pid_init(&pid, 1000000, -1000000, 0, 999999);
	CHECK_EQ(pid.kp_q8, MOTION_PID_GAIN_MAX);
	CHECK_EQ(pid.ki_q8, -MOTION_PID_GAIN_MAX);
	CHECK_EQ(pid.out_limit, MOTION_PID_OUT_MAX);
}

static void test_pid_antiwindup(void)
{
	motion_pid_t pid;
	int i, steps;
	int32_t out = 0;

	/* Hold a large error for a long time with the output saturated. */
	motion_pid_init(&pid, 256, 64, 0, 500);
	for (i = 0; i < 5000; i++) {
		out = motion_pid_step(&pid, 1000, 0);
	}
	CHECK_EQ(out, 500);
	/* The integral is bounded by the output range. */
	CHECK(pid.integ_q8 <= 500 * 256);

	/* Reverse the error: a wound-up controller would stay pinned at
	   +500 for thousands of steps; this one must leave saturation fast. */
	for (steps = 1; steps <= 100; steps++) {
		out = motion_pid_step(&pid, 0, 200);
		if (out < 500) {
			break;
		}
	}
	CHECK(steps <= 5);

	/* Conditional integration: while P alone saturates the output the
	   integral must not grow, so the first small overshoot pulls the
	   output straight down instead of leaving it pinned high. */
	motion_pid_init(&pid, 256, 64, 0, 500);
	for (i = 0; i < 2000; i++) {
		(void)motion_pid_step(&pid, 1000, 0);	/* P = 1000 > 500 */
	}
	CHECK_EQ(pid.integ_q8, 0);
	out = motion_pid_step(&pid, 100, 101);		/* error -1 */
	CHECK(out <= 0);
}

/* Extreme inputs must not overflow (UBSan would abort the run). */
static void test_pid_extremes(void)
{
	motion_pid_t pid;
	int i;
	int32_t out;

	motion_pid_init(&pid, MOTION_PID_GAIN_MAX, MOTION_PID_GAIN_MAX,
			MOTION_PID_GAIN_MAX, MOTION_PID_OUT_MAX);
	for (i = 0; i < 1000; i++) {
		int32_t sp = (i & 1) ? INT32_MAX : INT32_MIN;
		int32_t pv = (i & 1) ? INT32_MIN : INT32_MAX;
		out = motion_pid_step(&pid, sp, pv);
		CHECK(out <= MOTION_PID_OUT_MAX && out >= -MOTION_PID_OUT_MAX);
	}
	motion_pid_init(&pid, -MOTION_PID_GAIN_MAX, -MOTION_PID_GAIN_MAX,
			-MOTION_PID_GAIN_MAX, MOTION_PID_OUT_MAX);
	for (i = 0; i < 1000; i++) {
		out = motion_pid_step(&pid, (i % 3 == 0) ? 4096 : -4096, 0);
		CHECK(out <= MOTION_PID_OUT_MAX && out >= -MOTION_PID_OUT_MAX);
	}
}

/* Closed loop on a toy motor: speed = duty * 0.1 with first-order lag,
   plus a dead band.  A PI controller must reach the setpoint with no
   steady-state error and never exceed its output limit. */
static void test_pid_closed_loop(void)
{
	motion_pid_t pid;
	double speed = 0.0;
	int32_t duty = 0, measured = 0;
	int tick;

	motion_pid_init(&pid, 512, 128, 0, 600);
	for (tick = 0; tick < 500; tick++) {
		double drive;

		measured = (int32_t)(speed + 0.5);
		duty = motion_pid_step(&pid, 30, measured);
		CHECK(duty <= 600 && duty >= -600);

		drive = (duty > 80 || duty < -80) ? duty * 0.1 : 0.0;
		speed += (drive - speed) * 0.2;
	}
	CHECK_EQ(measured, 30);
}

/* ------------------------------------------------------------------ */

static void test_helpers(void)
{
	CHECK_EQ(motion_clamp(5, 0, 10), 5);
	CHECK_EQ(motion_clamp(-5, 0, 10), 0);
	CHECK_EQ(motion_clamp(15, 0, 10), 10);
	CHECK_EQ(motion_abs(-7), 7);
	CHECK_EQ(motion_abs(7), 7);
	CHECK_EQ(motion_abs(INT32_MIN), INT32_MAX);
}

/* ------------------------------------------------------------------ *
 *  distance_state (motion_ds.h and the motion_ds_* arithmetic)
 * ------------------------------------------------------------------ */

#define PI_D		3.14159265358979323846
#define MM_PER_COUNT	(PI_D * MOTION_WHEEL_DIA_MM / MOTION_COUNTS_PER_REV)
#define WINDOW_S	((double)MOTION_DS_WINDOW_TICKS * MOTION_TICK_MS / 1000.0)

/* Nearest whole number, halves away from zero (as the library rounds). */
static long long
round_ref (double x)
{
	return (long long)((x >= 0.0) ? (x + 0.5) : (x - 0.5));
}

static void test_ds_layout(void)
{
	uint32_t w;

	/* Each field lands in its own bits and nowhere else. */
	CHECK_EQ(motion_ds_pack(0x7FFF, MOTION_DS_STEADY, 0u, -2048), 0x00007FFFu);
	CHECK_EQ(motion_ds_pack(0, (motion_ds_state_t)3, 0u, -2048), 0x00018000u);
	CHECK_EQ(motion_ds_pack(0, MOTION_DS_STEADY, 7u, -2048), 0x000E0000u);
	CHECK_EQ(motion_ds_pack(0, MOTION_DS_STEADY, 0u, 2047), 0xFFF00000u);

	/* The word before any publish decodes to all zeros. */
	CHECK_EQ(motion_ds_turn_dps(MOTION_DS_WORD_ZERO), 0);
	CHECK_EQ(motion_ds_mm(MOTION_DS_WORD_ZERO), 0);
	CHECK_EQ(motion_ds_counter(MOTION_DS_WORD_ZERO), 0);
	CHECK_EQ(motion_ds_state(MOTION_DS_WORD_ZERO), MOTION_DS_STEADY);
	CHECK_EQ(motion_ds_pack(0, MOTION_DS_STEADY, 0u, 0), MOTION_DS_WORD_ZERO);

	/* Turn rate: offset 2048, clamped, sign kept. */
	w = motion_ds_pack(0, MOTION_DS_STEADY, 0u, -1);
	CHECK_EQ(motion_ds_turn_dps(w), -1);
	CHECK_EQ(motion_ds_turn_dps(motion_ds_pack(0, MOTION_DS_STEADY, 0u, 5000)), 2047);
	CHECK_EQ(motion_ds_turn_dps(motion_ds_pack(0, MOTION_DS_STEADY, 0u, -5000)), -2048);
	CHECK_EQ(motion_ds_turn_dps(motion_ds_pack(0, MOTION_DS_STEADY, 0u, INT32_MIN)), -2048);

	/* dist_mm is the net distance modulo 32768; a negative one wraps the
	   same way.  Counter keeps 3 bits. */
	CHECK_EQ(motion_ds_mm(motion_ds_pack(32768, MOTION_DS_STEADY, 0u, 0)), 0);
	CHECK_EQ(motion_ds_mm(motion_ds_pack(32769, MOTION_DS_STEADY, 0u, 0)), 1);
	CHECK_EQ(motion_ds_mm(motion_ds_pack(-1, MOTION_DS_STEADY, 0u, 0)), 32767);
	CHECK_EQ(motion_ds_counter(motion_ds_pack(0, MOTION_DS_STEADY, 9u, 0)), 1);
	CHECK_EQ(motion_ds_state(motion_ds_pack(0, MOTION_DS_DECEL, 0u, 0)), MOTION_DS_DECEL);
}

static void test_ds_round_trip(void)
{
	uint32_t i;

	srand(11u);
	for (i = 0u; i < 20000u; i++) {
		int32_t mm = (int32_t)(rand() % 200001) - 100000;
		motion_ds_state_t st = (motion_ds_state_t)(rand() % 3);
		uint32_t ctr = (uint32_t)rand();
		int32_t turn = (int32_t)(rand() % 4096) - 2048;
		uint32_t w = motion_ds_pack(mm, st, ctr, turn);

		CHECK_EQ(motion_ds_mm(w), (uint32_t)mm & 0x7FFFu);
		CHECK_EQ(motion_ds_state(w), st);
		CHECK_EQ(motion_ds_counter(w), ctr & 7u);
		CHECK_EQ(motion_ds_turn_dps(w), turn);
	}
}

static void test_ds_delta_and_updates(void)
{
	uint32_t a = motion_ds_pack(100, MOTION_DS_STEADY, 0u, 0);
	uint32_t b = motion_ds_pack(150, MOTION_DS_STEADY, 1u, 0);
	long long truth = 0, summed = 0;
	uint32_t prev = MOTION_DS_WORD_ZERO;
	int i;

	CHECK_EQ(motion_ds_delta_mm(b, a), 50);
	CHECK_EQ(motion_ds_delta_mm(a, b), -50);		/* reversing */
	CHECK_EQ(motion_ds_delta_mm(motion_ds_pack(5, MOTION_DS_STEADY, 0u, 0),
				    motion_ds_pack(32760, MOTION_DS_STEADY, 0u, 0)), 13);
	CHECK_EQ(motion_ds_delta_mm(motion_ds_pack(32760, MOTION_DS_STEADY, 0u, 0),
				    motion_ds_pack(5, MOTION_DS_STEADY, 0u, 0)), -13);
	CHECK_EQ(motion_ds_delta_mm(motion_ds_pack(16383, MOTION_DS_STEADY, 0u, 0),
				    motion_ds_pack(0, MOTION_DS_STEADY, 0u, 0)), 16383);
	CHECK_EQ(motion_ds_delta_mm(motion_ds_pack(16384, MOTION_DS_STEADY, 0u, 0),
				    motion_ds_pack(0, MOTION_DS_STEADY, 0u, 0)), -16384);
	/* Other fields never leak into the distance. */
	CHECK_EQ(motion_ds_delta_mm(motion_ds_pack(7, MOTION_DS_DECEL, 7u, -2048),
				    motion_ds_pack(7, MOTION_DS_ACCEL, 0u, 2047)), 0);

	CHECK_EQ(motion_ds_updates(b, a), 1);
	CHECK_EQ(motion_ds_updates(a, a), 0);
	CHECK_EQ(motion_ds_updates(motion_ds_pack(0, MOTION_DS_STEADY, 1u, 0),
				   motion_ds_pack(0, MOTION_DS_STEADY, 7u, 0)), 2);

	/* Random walk of the net distance, forward and back: the summed
	   deltas between reads equal the true distance. */
	srand(13u);
	for (i = 0; i < 200000; i++) {
		uint32_t now;

		truth += (rand() % 3001) - 1500;		/* |step| < 16384 */
		now = motion_ds_pack((int32_t)truth, MOTION_DS_STEADY, (uint32_t)i, 0);
		summed += motion_ds_delta_mm(now, prev);
		prev = now;
	}
	CHECK_EQ(summed, truth);
}

static void test_ds_dist(void)
{
	uint32_t i;

	CHECK_EQ(motion_ds_dist_mm(0, 0), 0);
	CHECK_EQ(motion_ds_dist_mm(100, -100), 0);	/* turn on the spot */
	/* One wheel turn on both wheels = pi x 64 = 201.06 mm. */
	CHECK_EQ(motion_ds_dist_mm(MOTION_COUNTS_PER_REV, MOTION_COUNTS_PER_REV), 201);
	CHECK_EQ(motion_ds_dist_mm(-MOTION_COUNTS_PER_REV, -MOTION_COUNTS_PER_REV), -201);
	/* 170 km: no overflow, and pi as 355/113 stays within 1 part in a
	   million (about 15 mm here). */
	{
		double ref = (double)INT32_MAX * MM_PER_COUNT;
		double got = (double)motion_ds_dist_mm(INT32_MAX, INT32_MAX);
		CHECK((got > ref * (1.0 - 1e-6)) && (got < ref * (1.0 + 1e-6)));
		got = (double)motion_ds_dist_mm(INT32_MIN, INT32_MIN);
		CHECK((got < -ref * (1.0 - 1e-6)) && (got > -ref * (1.0 + 1e-6) - 1.0));
	}

	srand(17u);
	for (i = 0u; i < 20000u; i++) {
		int32_t l = (int32_t)((rand() % 2000001) - 1000000) * 100;
		int32_t r = l + (int32_t)(rand() % 20001) - 10000;
		double ref = ((double)l + (double)r) / 2.0 * MM_PER_COUNT;
		long long got = motion_ds_dist_mm(l, r);

		/* pi as 355/113 is off by 1 part in 11 million */
		CHECK((got - round_ref(ref) <= 1) && (round_ref(ref) - got <= 1));
	}
}

/* Sum of the newest window of d_r - d_l, as deg/s. */
static double
turn_ref (double diff_counts)
{
	return diff_counts * MM_PER_COUNT / MOTION_WHEELBASE_MM
	       * (180.0 / PI_D) / WINDOW_S;
}

static void test_ds_turn(void)
{
	motion_ds_win_t w;
	uint32_t i;

	motion_ds_win_reset(&w);
	CHECK_EQ(motion_ds_win_turn_dps(&w), 0);

	/* Right wheel ahead = turning left = counter-clockwise = positive. */
	for (i = 0u; i < MOTION_DS_WINDOW_TICKS; i++) {
		motion_ds_win_add(&w, -13, 13);
	}
	CHECK(motion_ds_win_turn_dps(&w) > 0);
	CHECK_EQ(motion_ds_win_turn_dps(&w), round_ref(turn_ref(26.0 * MOTION_DS_WINDOW_TICKS)));

	for (i = 0u; i < MOTION_DS_WINDOW_TICKS; i++) {
		motion_ds_win_add(&w, 13, -13);
	}
	CHECK_EQ(motion_ds_win_turn_dps(&w), -round_ref(turn_ref(26.0 * MOTION_DS_WINDOW_TICKS)));

	/* Only the newest window counts: one older tick drops out. */
	motion_ds_win_reset(&w);
	motion_ds_win_add(&w, 0, 100);
	for (i = 0u; i < MOTION_DS_WINDOW_TICKS - 1u; i++) {
		motion_ds_win_add(&w, 0, 0);
	}
	CHECK_EQ(motion_ds_win_turn_dps(&w), round_ref(turn_ref(100.0)));
	motion_ds_win_add(&w, 0, 0);
	CHECK_EQ(motion_ds_win_turn_dps(&w), 0);

	/* One count of difference: the step size quoted in motion_cfg.h. */
	motion_ds_win_reset(&w);
	motion_ds_win_add(&w, 0, 1);
	printf("   distance_state: 1 count over %u ticks = %.3f deg/s (calculated)\n",
	       (unsigned)MOTION_DS_WINDOW_TICKS, turn_ref(1.0));
	CHECK_EQ(motion_ds_win_turn_dps(&w), round_ref(turn_ref(1.0)));

	/* Random windows against the floating-point formula. */
	srand(19u);
	for (i = 0u; i < 20000u; i++) {
		uint32_t k;
		double diff = 0.0;

		for (k = 0u; k < MOTION_DS_HIST; k++) {
			int32_t l = (rand() % 201) - 100, r = (rand() % 201) - 100;
			motion_ds_win_add(&w, l, r);
			if (k >= MOTION_DS_HIST - MOTION_DS_WINDOW_TICKS) {
				diff += (double)(r - l);
			}
		}
		CHECK_EQ(motion_ds_win_turn_dps(&w), round_ref(turn_ref(diff)));
	}

	/* A huge tick is clamped, never overflows. */
	motion_ds_win_reset(&w);
	motion_ds_win_add(&w, INT32_MIN, INT32_MAX);
	CHECK(motion_ds_win_turn_dps(&w) > 0);
}

/* Change in the summed forward counts between the two windows, as
   mm/s^2. */
static double
accel_ref (double fwd_change)
{
	return fwd_change / 2.0 * MM_PER_COUNT / (WINDOW_S * WINDOW_S);
}

static void test_ds_accel(void)
{
	motion_ds_win_t w;
	uint32_t i;
	int32_t a;

	motion_ds_win_reset(&w);
	CHECK_EQ(motion_ds_win_accel_mmps2(&w), 0);
	CHECK_EQ(motion_ds_accel_state(0), MOTION_DS_STEADY);
	CHECK_EQ(motion_ds_accel_state(MOTION_DS_ACCEL_MMPS2), MOTION_DS_ACCEL);
	CHECK_EQ(motion_ds_accel_state(MOTION_DS_ACCEL_MMPS2 - 1), MOTION_DS_STEADY);
	CHECK_EQ(motion_ds_accel_state(-MOTION_DS_ACCEL_MMPS2), MOTION_DS_DECEL);
	CHECK_EQ(motion_ds_accel_state(-MOTION_DS_ACCEL_MMPS2 + 1), MOTION_DS_STEADY);

	/* From rest to 13 counts per tick per wheel: accelerating until both
	   windows hold the new speed, then steady. */
	for (i = 1u; i <= MOTION_DS_HIST; i++) {
		motion_ds_win_add(&w, 13, 13);
		a = motion_ds_win_accel_mmps2(&w);
		if (i < MOTION_DS_HIST) {
			CHECK(motion_ds_accel_state(a) == MOTION_DS_ACCEL);
		} else {
			CHECK_EQ(a, 0);
		}
	}
	/* Stop: decelerating, then steady again. */
	for (i = 1u; i <= MOTION_DS_HIST; i++) {
		motion_ds_win_add(&w, 0, 0);
		a = motion_ds_win_accel_mmps2(&w);
		CHECK((i < MOTION_DS_HIST) ? (motion_ds_accel_state(a) == MOTION_DS_DECEL)
					   : (a == 0));
	}
	/* Turning on the spot is not forward acceleration. */
	for (i = 1u; i <= MOTION_DS_HIST; i++) {
		motion_ds_win_add(&w, -20, 20);
		CHECK_EQ(motion_ds_win_accel_mmps2(&w), 0);
	}

	/* Random histories against the floating-point formula. */
	srand(23u);
	for (i = 0u; i < 20000u; i++) {
		uint32_t k;
		double f_new = 0.0, f_old = 0.0;

		for (k = 0u; k < MOTION_DS_HIST; k++) {
			int32_t l = (rand() % 201) - 100, r = (rand() % 201) - 100;
			motion_ds_win_add(&w, l, r);
			if (k >= MOTION_DS_HIST - MOTION_DS_WINDOW_TICKS) {
				f_new += (double)(l + r);
			} else {
				f_old += (double)(l + r);
			}
		}
		a = motion_ds_win_accel_mmps2(&w);
		CHECK((a - round_ref(accel_ref(f_new - f_old)) <= 1)
		      && (round_ref(accel_ref(f_new - f_old)) - a <= 1));
	}

	/* Extremes saturate instead of overflowing. */
	motion_ds_win_reset(&w);
	for (i = 0u; i < MOTION_DS_WINDOW_TICKS; i++) {
		motion_ds_win_add(&w, INT32_MAX, INT32_MAX);
	}
	CHECK(motion_ds_win_accel_mmps2(&w) > 0);
}

int main(void)
{
	test_quad_table_properties();
	test_quad_counting();
	test_quad_invalid();
	test_quad_ab_swap();
	test_pack();
	test_delta();
	test_delta_random_walk();
	test_stall_basics();
	test_stall_dead_phase();
	test_stall_wire_comes_off();
	test_stall_healthy_then_dead();
	test_pid_basic();
	test_pid_antiwindup();
	test_pid_extremes();
	test_pid_closed_loop();
	test_helpers();
	test_ds_layout();
	test_ds_round_trip();
	test_ds_delta_and_updates();
	test_ds_dist();
	test_ds_turn();
	test_ds_accel();

	printf("motion_lib: %d checks, %d failures\n", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
