/* Host test for line_state.h, and the contract with motion_task: inputs built
   from line_state words drive Buddy 2's real statechart (motion_sm.c) through
   follow -> junction turn -> follow, and to a search when the line is lost. */

#include "check.h"
#include "../../app_program/line_task/line_state.h"
#include "../../app_program/line_task/barcode_action.h"
#include "../../app_program/motor_task/motion_sm.h"

static uint32_t word_of(uint32_t rl, bool barcode, bool lost, uint32_t ctr,
			int32_t error, uint32_t dark_ms)
{
	line_state_fields_t f = {
		.rl = rl, .steer = line_state_steer_for(rl, false),
		.barcode = barcode, .lost = lost, .fault = false, .cal = true,
		.counter = ctr, .error = error, .dark_ms = dark_ms,
	};
	return line_state_pack(&f);
}

/* What motion_task would compute from one line_state read. */
static void inputs_from(uint32_t word, motion_sm_in_t *p_in)
{
	p_in->b_line_seen    = (line_state_rl(word) != LINE_RL_CENTRED) ||
			       (line_state_dark_ms(word) < 50u);
	p_in->b_line_missing = line_state_lost(word) || line_state_fault(word);
	p_in->b_line_found   = line_state_rl(word) != LINE_RL_CENTRED;
}

static void test_pack(void)
{
	line_state_fields_t f = {
		.rl = LINE_RL_JUNCTION, .steer = LINE_STEER_RIGHT, .barcode = true,
		.lost = true, .fault = true, .cal = true, .counter = 0x1ABu,
		.error = -150, .dark_ms = 70000u,
	};
	uint32_t word = line_state_pack(&f);

	CHECK(line_state_rl(word) == LINE_RL_JUNCTION);
	CHECK(line_state_steer(word) == LINE_STEER_RIGHT);
	CHECK(line_state_barcode(word) && line_state_lost(word));
	CHECK(line_state_fault(word) && line_state_cal(word));
	CHECK(line_state_counter(word) == 0xABu);
	CHECK(line_state_error(word) == -100);			/* clamped */
	CHECK(line_state_dark_ms(word) == 2550u);		/* saturates */

	word = word_of(LINE_RL_CENTRED, false, false, 0u, 37, 120u);
	CHECK(!line_state_barcode(word) && !line_state_lost(word));
	CHECK(!line_state_fault(word));
	CHECK(line_state_error(word) == 37);
	CHECK(line_state_dark_ms(word) == 120u);
	CHECK(line_state_lost(LINE_STATE_INITIAL));
	CHECK(line_state_error(LINE_STATE_INITIAL) == 0);

	/* the line-following spec: GP26 black corrects right, GP27 left */
	CHECK(line_state_steer_for(LINE_RL_LEFT_DEV, false) == LINE_STEER_RIGHT);
	CHECK(line_state_steer_for(LINE_RL_RIGHT_DEV, false) == LINE_STEER_LEFT);
	CHECK(line_state_steer_for(LINE_RL_LEFT_DEV, true) == LINE_STEER_LEFT);
	CHECK(line_state_steer_for(LINE_RL_CENTRED, false) == LINE_STEER_NONE);
	CHECK(line_state_steer_for(LINE_RL_JUNCTION, false) == LINE_STEER_NONE);
}

static void test_motion_contract(void)
{
	motion_sm_t sm;
	motion_sm_in_t in = { 0 };
	motion_sm_out_t out;

	motion_sm_init(&sm);
	in.b_running = true;

	inputs_from(word_of(LINE_RL_LEFT_DEV, false, false, 1u, -70, 0u), &in);
	motion_sm_step(&sm, &in, &out);
	CHECK(motion_sm_state(&sm) == MOTION_ST_LINE_FOLLOW);

	/* centred for 1 s: still following */
	inputs_from(word_of(LINE_RL_CENTRED, false, false, 2u, 0, 1000u), &in);
	motion_sm_step(&sm, &in, &out);
	CHECK(motion_sm_state(&sm) == MOTION_ST_LINE_FOLLOW);

	/* a barcode under the car: never mistaken for lost */
	inputs_from(word_of(LINE_RL_CENTRED, true, false, 3u, 0, 0u), &in);
	motion_sm_step(&sm, &in, &out);
	CHECK(motion_sm_state(&sm) == MOTION_ST_LINE_FOLLOW);

	/* junction: nav sends JUNCTION(action) -> Turning */
	inputs_from(word_of(LINE_RL_JUNCTION, false, false, 4u, 0, 0u), &in);
	in.nav = MOTION_SM_NAV_JUNCTION;
	motion_sm_step(&sm, &in, &out);
	in.nav = MOTION_SM_NAV_NONE;
	CHECK(motion_sm_state(&sm) == MOTION_ST_TURNING);

	/* turn done on the new line: MOVE_DONE, back to following */
	inputs_from(word_of(LINE_RL_RIGHT_DEV, false, false, 5u, 70, 0u), &in);
	in.b_done = true;
	motion_sm_step(&sm, &in, &out);
	in.b_done = false;
	CHECK(motion_sm_state(&sm) == MOTION_ST_LINE_FOLLOW);
	CHECK((out.replies & MOTION_SM_REP_MOVE_DONE) != 0u);

	/* a second junction command while turning is refused, as nav expects */
	in.nav = MOTION_SM_NAV_JUNCTION;
	motion_sm_step(&sm, &in, &out);
	motion_sm_step(&sm, &in, &out);
	in.nav = MOTION_SM_NAV_NONE;
	CHECK(motion_sm_state(&sm) == MOTION_ST_TURNING);
	CHECK((out.replies & MOTION_SM_REP_CMD_REJECTED) != 0u);
	in.b_done = true;
	motion_sm_step(&sm, &in, &out);
	in.b_done = false;
	CHECK(motion_sm_state(&sm) == MOTION_ST_LINE_FOLLOW);

	/* lost (noise-filtered in ir_sense_task): Line Search */
	inputs_from(word_of(LINE_RL_CENTRED, false, true, 6u, 0, LINE_LOST_MS), &in);
	motion_sm_step(&sm, &in, &out);
	CHECK(motion_sm_state(&sm) == MOTION_ST_LINE_SEARCH);

	/* line touched again: LINE_FOUND, following */
	inputs_from(word_of(LINE_RL_LEFT_DEV, false, false, 7u, -70, 0u), &in);
	motion_sm_step(&sm, &in, &out);
	CHECK(motion_sm_state(&sm) == MOTION_ST_LINE_FOLLOW);
	CHECK((out.replies & MOTION_SM_REP_LINE_FOUND) != 0u);
}

int main(void)
{
	test_pack();
	test_motion_contract();
	CHECK((uint32_t)BARCODE_ACT_UTURN <= 3u);	/* fits the 2-bit command byte */
	return check_report("line state + motion contract");
}
