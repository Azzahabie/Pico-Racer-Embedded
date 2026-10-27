/*
 *	test_motion_sm.c
 *	Host unit tests for app_program/motor_task/motion_sm.c: every
 *	transition of design statechart 6.2 (T1-T20, listed in the Statechart
 *	section of docs/motion/README.md), the replies, the rules agreed with
 *	Buddy 5 for an interrupted turn, the priority within one step, and
 *	the packed state byte.
 *	    make -C tests/motion_sm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion_sm.h"

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

#define ST(x)		MOTION_ST_##x
#define REP(x)		MOTION_SM_REP_##x

static motion_sm_t	g_sm;
static motion_sm_in_t	g_in;		/* the inputs held between steps */
static motion_sm_out_t	g_out;

/* Mission Running, nothing else. */
static void
reset_in (void)
{
	memset(&g_in, 0, sizeof g_in);
	g_in.b_running = true;
}

/* One step with the held inputs; one-shot inputs are cleared after it. */
static void
step (void)
{
	motion_sm_step(&g_sm, &g_in, &g_out);
	g_in.avoid = MOTION_SM_AVOID_NONE;
	g_in.nav = MOTION_SM_NAV_NONE;
	g_in.b_done = false;
	g_in.b_line_found = false;
	g_in.b_line_missing = false;
	g_in.b_search_over = false;
	g_in.b_fault = false;
}

/* Fresh statechart, mission Running, first step into Line Following. */
static void
start_following (void)
{
	motion_sm_init(&g_sm);
	reset_in();
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
}

static void
start_turning (void)
{
	start_following();
	g_in.nav = MOTION_SM_NAV_JUNCTION;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(TURNING));
	CHECK_EQ(g_out.replies, 0);
}

/* ------------------------------------------------------------------ */

static void test_packing(void)
{
	uint32_t s;

	motion_sm_init(&g_sm);
	CHECK_EQ(motion_sm_state(&g_sm), ST(IDLE));
	CHECK_EQ(motion_sm_history(&g_sm), MOTION_ST_NONE);
	CHECK_EQ(g_sm.packed, 0xF0);		/* bits 0-3 state, 4-7 history */
	CHECK_EQ(sizeof g_sm, 2);

	for (s = 0u; s < (uint32_t)MOTION_ST_COUNT; s++) {
		CHECK(strcmp(motion_sm_name(s), "?") != 0);
	}
	CHECK(strcmp(motion_sm_name(MOTION_ST_COUNT), "?") == 0);
	CHECK(strcmp(motion_sm_name(ST(LINE_FOLLOW)), "LINE_FOLLOW") == 0);

	CHECK(motion_sm_driving(ST(LINE_FOLLOW)) && motion_sm_driving(ST(REVERSING)));
	CHECK(!motion_sm_driving(ST(BRAKED)) && !motion_sm_driving(ST(LINE_LOST)));
	CHECK(!motion_sm_driving(ST(IDLE)) && !motion_sm_driving(ST(FAULT)));
}

static void test_idle(void)
{
	/* T1: Running, no STOP bit -> Line Following. */
	motion_sm_init(&g_sm);
	reset_in();
	g_in.b_running = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(IDLE));
	CHECK(!g_out.b_changed);
	g_in.b_running = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
	CHECK(g_out.b_changed && !g_out.b_resumed);

	/* T2: Running with a STOP bit -> Braked, then the default on clear. */
	motion_sm_init(&g_sm);
	reset_in();
	g_in.b_stop_scan = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	CHECK_EQ(motion_sm_history(&g_sm), MOTION_ST_NONE);
	g_in.b_stop_scan = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));	/* T10 */

	/* T2 with STOP_IMU. */
	motion_sm_init(&g_sm);
	reset_in();
	g_in.b_stop_imu = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
}

static void test_stop_and_history(void)
{
	motion_state_t from[] = { ST(LINE_FOLLOW), ST(TURNING), ST(BYPASS),
				  ST(LINE_SEARCH), ST(LINE_LOST) };
	unsigned i;

	/* T5 and T9 from each Active state: braked, then back to it. */
	for (i = 0u; i < sizeof from / sizeof from[0]; i++) {
		motion_sm_init(&g_sm);
		g_sm.packed = (uint8_t)(0xF0u | (unsigned)from[i]);
		reset_in();
		g_in.b_stop_imu = (i % 2u) != 0u;	/* either STOP bit */
		g_in.b_stop_scan = !g_in.b_stop_imu;
		step();
		CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
		CHECK_EQ(motion_sm_history(&g_sm), from[i]);
		step();					/* still set: stays */
		CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
		g_in.b_stop_imu = g_in.b_stop_scan = false;
		step();
		CHECK_EQ(motion_sm_state(&g_sm), from[i]);
		CHECK(g_out.b_changed && g_out.b_resumed);
		CHECK_EQ(motion_sm_history(&g_sm), MOTION_ST_NONE);
		CHECK_EQ(g_out.replies, 0);
	}

	/* Both bits: stays braked until both are clear. */
	start_following();
	g_in.b_stop_scan = g_in.b_stop_imu = true;
	step();
	g_in.b_stop_scan = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	g_in.b_stop_imu = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
}

static void test_mission_leaves(void)
{
	motion_state_t from[] = { ST(LINE_FOLLOW), ST(BYPASS), ST(LINE_SEARCH),
				  ST(LINE_LOST), ST(BRAKED) };
	unsigned i;

	/* T3, T4: every state but Motion Fault goes to Idle, history cleared. */
	for (i = 0u; i < sizeof from / sizeof from[0]; i++) {
		motion_sm_init(&g_sm);
		g_sm.packed = (uint8_t)(((unsigned)ST(LINE_FOLLOW) << 4) | (unsigned)from[i]);
		g_sm.flags = MOTION_SM_F_AFTER_BYPASS | MOTION_SM_F_BYPASS_WAIT;
		reset_in();
		g_in.b_running = false;
		step();
		CHECK_EQ(motion_sm_state(&g_sm), ST(IDLE));
		CHECK_EQ(motion_sm_history(&g_sm), MOTION_ST_NONE);
		CHECK_EQ(g_sm.flags, 0);
		CHECK_EQ(g_out.replies, 0);
	}

	/* Reversing -> Idle still sends REVERSE_DONE. */
	motion_sm_init(&g_sm);
	g_sm.packed = (uint8_t)(0xF0u | (unsigned)ST(REVERSING));
	reset_in();
	g_in.b_running = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(IDLE));
	CHECK_EQ(g_out.replies, REP(REVERSE_DONE));

	/* Motion Fault stays when the mission leaves Running. */
	motion_sm_init(&g_sm);
	g_sm.packed = (uint8_t)(0xF0u | (unsigned)ST(FAULT));
	reset_in();
	g_in.b_running = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(FAULT));
}

static void test_turn(void)
{
	/* T11, T12: turn, target reached with the line -> MOVE_DONE. */
	start_turning();
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(TURNING));
	g_in.b_done = true;
	g_in.b_line_seen = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
	CHECK_EQ(g_out.replies, REP(MOVE_DONE));
	CHECK_EQ(g_sm.flags, 0);

	/* T13: no line at the target -> Line Search, still MOVE_DONE. */
	start_turning();
	g_in.b_done = true;
	g_in.b_line_seen = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_SEARCH));
	CHECK_EQ(g_out.replies, REP(MOVE_DONE));

	/* I1: a junction command anywhere but Line Following is refused. */
	start_turning();
	g_in.nav = MOTION_SM_NAV_JUNCTION;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(TURNING));
	CHECK_EQ(g_out.replies, REP(CMD_REJECTED));
	motion_sm_init(&g_sm);
	reset_in();
	g_in.b_running = false;
	g_in.nav = MOTION_SM_NAV_JUNCTION;
	step();
	CHECK_EQ(g_out.replies, REP(CMD_REJECTED));
}

/* The rules agreed with Buddy 5 (PostTri1 Version C row 15). */
static void test_interrupted_turn(void)
{
	/* False alarm: STOP_SCAN cleared, nothing queued -> the turn resumes
	   and then sends MOVE_DONE, once. */
	start_turning();
	g_in.b_stop_scan = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	CHECK_EQ(g_out.replies, 0);
	g_in.b_stop_scan = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(TURNING));
	CHECK(g_out.b_resumed);
	g_in.b_done = true;
	g_in.b_line_seen = true;
	step();
	CHECK_EQ(g_out.replies, REP(MOVE_DONE));

	/* A reverse arrives: the turn is dropped with one CMD_REJECTED, and
	   after the reverse and the clear the car follows the line again. */
	start_turning();
	g_in.b_stop_scan = true;
	step();
	g_in.avoid = MOTION_SM_AVOID_REVERSE;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(REVERSING));
	CHECK_EQ(g_out.replies, REP(CMD_REJECTED));
	g_in.b_done = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	CHECK_EQ(g_out.replies, REP(REVERSE_DONE));
	g_in.b_stop_scan = false;			/* false alarm on rescan */
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
	CHECK_EQ(g_out.replies, 0);			/* no second reply */

	/* A bypass arrives: dropped with one CMD_REJECTED, then Bypass. */
	start_turning();
	g_in.b_stop_scan = true;
	step();
	g_in.avoid = MOTION_SM_AVOID_BYPASS;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));	/* STOP still set */
	CHECK_EQ(g_out.replies, REP(CMD_REJECTED));
	g_in.b_stop_scan = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BYPASS));
	CHECK_EQ(g_out.replies, 0);

	/* The mission leaves Running mid-turn: one CMD_REJECTED. */
	start_turning();
	g_in.b_running = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(IDLE));
	CHECK_EQ(g_out.replies, REP(CMD_REJECTED));

	/* ... and while braked in the middle of a turn, too. */
	start_turning();
	g_in.b_stop_imu = true;
	step();
	g_in.b_running = false;
	step();
	CHECK_EQ(g_out.replies, REP(CMD_REJECTED));
}

static void test_reverse_and_bypass(void)
{
	/* T6, T7: reverse while STOP_SCAN stays set; REVERSE_DONE at the
	   distance, back to Braked. */
	start_following();
	g_in.b_stop_scan = true;
	step();
	g_in.avoid = MOTION_SM_AVOID_REVERSE;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(REVERSING));
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(REVERSING));	/* STOP_SCAN set */
	g_in.b_done = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	CHECK_EQ(g_out.replies, REP(REVERSE_DONE));

	/* T7: STOP_IMU during a reverse ends it, with REVERSE_DONE. */
	start_following();
	g_in.b_stop_scan = true;
	step();
	g_in.avoid = MOTION_SM_AVOID_REVERSE;
	step();
	g_in.b_stop_imu = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	CHECK_EQ(g_out.replies, REP(REVERSE_DONE));

	/* A reverse while STOP_IMU is set is refused, but scan_task is still
	   told it is over (REVIEW D12). */
	g_in.avoid = MOTION_SM_AVOID_REVERSE;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	CHECK_EQ(g_out.replies, REP(REVERSE_DONE));

	/* ... and it ends an interrupted turn with one CMD_REJECTED. */
	start_turning();
	g_in.b_stop_scan = g_in.b_stop_imu = true;
	step();
	g_in.avoid = MOTION_SM_AVOID_REVERSE;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	CHECK_EQ(g_out.replies, REP(REVERSE_DONE) | REP(CMD_REJECTED));
	g_in.b_stop_scan = g_in.b_stop_imu = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));	/* not the turn */
	CHECK_EQ(g_out.replies, 0);

	/* T8, T16, T15: bypass queued with STOP set, starts when it clears,
	   then a search whose LINE_FOUND also goes to scan_task. */
	start_following();
	g_in.b_stop_scan = true;
	step();
	g_in.avoid = MOTION_SM_AVOID_BYPASS;
	g_in.b_stop_scan = false;			/* same tick, as scan does */
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BYPASS));
	CHECK(!g_out.b_resumed);
	g_in.b_done = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_SEARCH));
	g_in.b_line_found = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
	CHECK_EQ(g_out.replies, REP(LINE_FOUND) | REP(TO_SCAN));

	/* A later, ordinary search reports to nav only. */
	g_in.b_line_missing = true;
	step();
	g_in.b_line_found = true;
	step();
	CHECK_EQ(g_out.replies, REP(LINE_FOUND));

	/* So does a search after a turn that followed a bypass search. */
	g_in.nav = MOTION_SM_NAV_JUNCTION;
	step();
	g_in.b_done = true;
	g_in.b_line_seen = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_SEARCH));
	g_in.b_line_found = true;
	step();
	CHECK_EQ(g_out.replies, REP(LINE_FOUND));

	/* T17 after a bypass: LINE_LOST to nav and scan, Line Lost. */
	start_following();
	g_in.b_stop_scan = true;
	step();
	g_in.avoid = MOTION_SM_AVOID_BYPASS;
	g_in.b_stop_scan = false;
	step();
	g_in.b_done = true;
	step();
	g_in.b_search_over = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_LOST));
	CHECK_EQ(g_out.replies, REP(LINE_LOST) | REP(TO_SCAN));
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_LOST));	/* waits for nav */
	g_in.b_running = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(IDLE));

	/* An avoidance command outside Braked changes nothing. */
	start_following();
	g_in.avoid = MOTION_SM_AVOID_REVERSE;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
	CHECK_EQ(g_out.replies, 0);
}

static void test_line_search(void)
{
	/* T14, T15 without a bypass: LINE_FOUND to nav only. */
	start_following();
	g_in.b_line_missing = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_SEARCH));
	g_in.b_line_found = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
	CHECK_EQ(g_out.replies, REP(LINE_FOUND));

	/* T17 without a bypass: LINE_LOST to nav only. */
	start_following();
	g_in.b_line_missing = true;
	step();
	g_in.b_search_over = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_LOST));
	CHECK_EQ(g_out.replies, REP(LINE_LOST));
}

static void test_fault(void)
{
	motion_state_t driving[] = { ST(LINE_FOLLOW), ST(BYPASS),
				     ST(LINE_SEARCH), ST(REVERSING) };
	unsigned i;

	/* T18, T19: a fault in any driving state; Reversing also gets
	   REVERSE_DONE. */
	for (i = 0u; i < sizeof driving / sizeof driving[0]; i++) {
		motion_sm_init(&g_sm);
		g_sm.packed = (uint8_t)(0xF0u | (unsigned)driving[i]);
		reset_in();
		g_in.b_stop_scan = (ST(REVERSING) == driving[i]);
		g_in.b_fault = true;
		step();
		CHECK_EQ(motion_sm_state(&g_sm), ST(FAULT));
		CHECK_EQ(g_out.replies, REP(MOTION_FAULT)
			 | ((ST(REVERSING) == driving[i]) ? REP(REVERSE_DONE) : 0u));
	}

	/* A fault mid-turn also drops the turn: one CMD_REJECTED. */
	start_turning();
	g_in.b_fault = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(FAULT));
	CHECK_EQ(g_out.replies, REP(MOTION_FAULT) | REP(CMD_REJECTED));
	CHECK_EQ(g_sm.flags, 0);

	/* Not driving: no fault transition. */
	motion_sm_init(&g_sm);
	reset_in();
	g_in.b_stop_scan = true;
	step();
	g_in.b_fault = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));

	/* T20: only FAULT_CLEAR(MOTION) leaves Motion Fault, to Idle. */
	start_following();
	g_in.b_fault = true;
	step();
	g_in.b_stop_scan = false;
	step();
	g_in.nav = MOTION_SM_NAV_CLEAR_IMU;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(FAULT));
	CHECK(g_out.b_clear_stop_imu);
	g_in.nav = MOTION_SM_NAV_CLEAR_MOTION;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(IDLE));
	CHECK_EQ(g_out.replies, 0);
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));	/* still Running */
}

static void test_clear_imu(void)
{
	/* I2: FAULT_CLEAR(IMU) asks for STOP_IMU to be cleared in any state;
	   the statechart then sees the bit clear on the next step. */
	start_following();
	g_in.b_stop_imu = true;
	step();
	g_in.nav = MOTION_SM_NAV_CLEAR_IMU;
	step();
	CHECK(g_out.b_clear_stop_imu);
	CHECK_EQ(g_out.replies, 0);
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	g_in.b_stop_imu = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(LINE_FOLLOW));
	step();
	CHECK(!g_out.b_clear_stop_imu);
}

/* Within one step: Motion Fault, then a new fault, then the mission,
   then a STOP bit, then the state's own rules. */
static void test_priority(void)
{
	start_following();
	g_in.b_fault = true;
	g_in.b_stop_scan = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(FAULT));
	CHECK_EQ(g_out.replies, REP(MOTION_FAULT));

	start_following();
	g_in.b_fault = true;
	g_in.b_running = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(FAULT));
	CHECK_EQ(g_out.replies, REP(MOTION_FAULT));

	start_following();
	g_in.b_stop_scan = true;
	g_in.b_running = false;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(IDLE));

	start_turning();
	g_in.b_fault = true;
	g_in.b_stop_scan = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(FAULT));
	CHECK_EQ(g_out.replies, REP(MOTION_FAULT) | REP(CMD_REJECTED));
}

/* I1 in every state but Line Following, and on the tick a STOP arrives. */
static void test_junction_refused(void)
{
	motion_state_t states[] = { ST(IDLE), ST(TURNING), ST(BYPASS),
				    ST(LINE_SEARCH), ST(LINE_LOST), ST(BRAKED),
				    ST(REVERSING), ST(FAULT) };
	unsigned i;

	for (i = 0u; i < sizeof states / sizeof states[0]; i++) {
		motion_sm_init(&g_sm);
		g_sm.packed = (uint8_t)(0xF0u | (unsigned)states[i]);
		reset_in();
		g_in.b_running = (ST(IDLE) != states[i]);
		g_in.b_stop_scan = (ST(BRAKED) == states[i]) || (ST(REVERSING) == states[i]);
		g_in.nav = MOTION_SM_NAV_JUNCTION;
		step();
		CHECK_EQ(motion_sm_state(&g_sm), states[i]);
		CHECK_EQ(g_out.replies, REP(CMD_REJECTED));
	}

	start_following();
	g_in.nav = MOTION_SM_NAV_JUNCTION;
	g_in.b_stop_scan = true;
	step();
	CHECK_EQ(motion_sm_state(&g_sm), ST(BRAKED));
	CHECK_EQ(g_out.replies, REP(CMD_REJECTED));
}

int main(void)
{
	test_packing();
	test_idle();
	test_stop_and_history();
	test_mission_leaves();
	test_turn();
	test_interrupted_turn();
	test_reverse_and_bypass();
	test_line_search();
	test_fault();
	test_clear_imu();
	test_priority();
	test_junction_refused();

	printf("motion_sm: %d checks, %d failures\n", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
