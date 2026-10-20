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

static void test_stall(void)
{
	motion_stall_t s;
	int i;

	motion_stall_init(&s, 30u, 3);

	/* Not driven: never a stall, however long. */
	for (i = 0; i < 1000; i++) {
		CHECK(!motion_stall_update(&s, false, 0));
	}

	/* Driven with no movement: stall on exactly the 30th tick. */
	for (i = 1; i < 30; i++) {
		CHECK(!motion_stall_update(&s, true, 0));
	}
	CHECK(motion_stall_update(&s, true, 0));

	/* Moving well: never a stall. */
	motion_stall_reset(&s);
	for (i = 0; i < 1000; i++) {
		CHECK(!motion_stall_update(&s, true, 5));
	}

	/* One count per 10 ticks reaches 3 counts by tick 21 < 30: no stall. */
	motion_stall_reset(&s);
	for (i = 1; i <= 300; i++) {
		CHECK(!motion_stall_update(&s, true, (i % 10 == 0) ? 1 : 0));
	}

	/* One count per 15 ticks needs 45 ticks for 3 counts: stalls. */
	motion_stall_reset(&s);
	{
		bool b_hit = false;
		for (i = 1; i <= 45 && !b_hit; i++) {
			b_hit = motion_stall_update(&s, true, (i % 15 == 0) ? 1 : 0);
		}
		CHECK(b_hit);
		CHECK_EQ(i - 1, 30);
	}

	/* Negative progress (wrong direction) is not progress. */
	motion_stall_reset(&s);
	for (i = 1; i < 30; i++) {
		CHECK(!motion_stall_update(&s, true, -50));
	}
	CHECK(motion_stall_update(&s, true, -50));

	/* Stopping the drive clears the window. */
	motion_stall_reset(&s);
	for (i = 1; i < 30; i++) {
		(void)motion_stall_update(&s, true, 0);
	}
	CHECK(!motion_stall_update(&s, false, 0));
	CHECK(!motion_stall_update(&s, true, 0));
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

int main(void)
{
	test_quad_table_properties();
	test_quad_counting();
	test_quad_invalid();
	test_quad_ab_swap();
	test_pack();
	test_delta();
	test_delta_random_walk();
	test_stall();
	test_pid_basic();
	test_pid_antiwindup();
	test_pid_extremes();
	test_pid_closed_loop();
	test_helpers();

	printf("motion_lib: %d checks, %d failures\n", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
