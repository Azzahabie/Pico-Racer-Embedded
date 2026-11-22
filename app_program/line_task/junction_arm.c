/*
 * junction_arm.c - the barcode-armed junction, one call per 1 ms sample.
 */

#include "junction_arm.h"
#include "barcode_action.h"
#include "barcode_rx.h"

#define RUN_MAX (100000u)	/* saturation for the run counters */

void junction_arm_init(junction_arm_t *arm)
{
	arm->phase = JARM_IDLE;
	arm->was_armed = false;
	arm->read_id = 0u;
	arm->letter = '\0';
	arm->seq_before = 0u;
	arm->waited = 0u;
	arm->cmd = BARCODE_CMD_NONE;
	arm->travel_mm = 0u;
	arm->black_ms = 0u;
	arm->black_mm = 0u;
}

void junction_arm_read_done(junction_arm_t *arm, uint8_t read_id, char letter,
			    uint32_t cmd_before)
{
	arm->was_armed = (arm->phase == JARM_ARMED) ||
			 ((arm->phase == JARM_WAIT) && arm->was_armed);
	arm->phase = JARM_WAIT;
	arm->read_id = read_id;
	arm->letter = letter;
	arm->seq_before = barcode_cmd_seq(cmd_before);
	arm->waited = 0u;
}

static uint32_t sat_add(uint32_t a, uint32_t b)
{
	return ((RUN_MAX - a) < b) ? RUN_MAX : a + b;
}

/* WAIT: has barcode_task judged the read, and did it store a command? */
static jarm_event_t wait_step(junction_arm_t *arm, uint32_t judged,
			      uint32_t cmd)
{
	if (barcode_judged_is(judged, arm->read_id)) {
		if ((barcode_cmd_letter(cmd) != BARCODE_LETTER_NONE) &&
		    (barcode_cmd_seq(cmd) != arm->seq_before)) {
			arm->phase = JARM_ARMED;
			arm->cmd = cmd;
			arm->travel_mm = 0u;
			return JARM_EV_ARMED;
		}
		/* rejected: no command stored (the stored one, if any, stays) */
		arm->phase = arm->was_armed ? JARM_ARMED : JARM_IDLE;
		return JARM_EV_REJECTED;
	}
	if (++arm->waited >= LINE_ARMED_VERDICT_WAIT_MS) {
		arm->phase = arm->was_armed ? JARM_ARMED : JARM_IDLE;
		return JARM_EV_NO_VERDICT;
	}
	return JARM_EV_NONE;
}

jarm_event_t junction_arm_step(junction_arm_t *arm, uint32_t judged,
			       uint32_t cmd, bool black, uint32_t moved_mm,
			       bool in_read, line_junction_gate_t *gate)
{
	jarm_event_t event = JARM_EV_NONE;

	/* GP7's current black run, in samples and in travel */
	if (black) {
		arm->black_ms = sat_add(arm->black_ms, 1u);
		arm->black_mm = sat_add(arm->black_mm, moved_mm);
	} else {
		arm->black_ms = 0u;
		arm->black_mm = 0u;
	}

	if (arm->phase == JARM_WAIT) {
		event = wait_step(arm, judged, cmd);
	} else if (arm->phase == JARM_ARMED) {
		arm->travel_mm = sat_add(arm->travel_mm, moved_mm);
		/* never in the middle of a black run: that may be the crossbar */
		if (!black && (arm->travel_mm >= LINE_ARMED_HOLD_MM)) {
			arm->phase = JARM_IDLE;
			event = JARM_EV_DISARM_DISTANCE;
		}
	} else {
		/* IDLE: the length rule alone */
	}

	gate->in_read = in_read;
	gate->wait = (arm->phase == JARM_WAIT);
	gate->armed = (arm->phase == JARM_ARMED);
	gate->armed_due = (arm->phase == JARM_ARMED) && black &&
			  (arm->black_ms >= LINE_ARMED_MIN_SAMPLES) &&
			  (arm->black_mm >= LINE_ARMED_CONFIRM_MM);
	return event;
}

bool junction_arm_used(junction_arm_t *arm)
{
	bool armed = (arm->phase == JARM_ARMED);

	if (armed) {
		arm->phase = JARM_IDLE;
	}
	return armed;
}

jarm_event_t junction_arm_cancel(junction_arm_t *arm)
{
	bool busy = (arm->phase != JARM_IDLE);

	arm->phase = JARM_IDLE;
	arm->was_armed = false;
	return busy ? JARM_EV_DISARM_MISSION : JARM_EV_NONE;
}
