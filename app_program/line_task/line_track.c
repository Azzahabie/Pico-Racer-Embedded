/*
 * line_track.c - one 1 ms step of line tracking.
 */

#include "line_track.h"

#define CANDIDATE_COUNT_MAX (1000u)

void line_track_disarm_junction(line_track_t *trk)
{
	trk->both_black_run = 0u;
	trk->clear_run = 0u;
	trk->junction_armed = false;
}

static void track_reset(line_track_t *trk)
{
	trk->left_black = false;
	trk->right_black = false;
	trk->filtered = LINE_RL_CENTRED;
	trk->candidate = LINE_RL_CENTRED;
	trk->candidate_count = 0u;
	trk->both_black_run = 0u;
	trk->clear_run = 0u;
	trk->junction_armed = false;	/* rearm after the sensors leave it */
	trk->dark_ms = LINE_ST_DARK_MASK * LINE_ST_DARK_UNIT_MS;
	trk->left_rail_run = 0u;
	trk->right_rail_run = 0u;
}

void line_track_init(line_track_t *trk, const ir_cal_sensor_t *left,
		     const ir_cal_sensor_t *right, bool steer_invert)
{
	trk->left_cal = *left;
	trk->right_cal = *right;
	trk->calibrated = false;
	trk->steer_invert = steer_invert;
	trk->counter = 0u;
	track_reset(trk);
}

void line_track_set_cal(line_track_t *trk, const ir_cal_sensor_t *left,
			const ir_cal_sensor_t *right)
{
	trk->left_cal = *left;
	trk->right_cal = *right;
	trk->calibrated = true;
}

/* The existing debounce: a new state after LINE_CONFIRM_SAMPLES, 00 after
   LINE_LOST_SAMPLES. */
static uint32_t debounce(line_track_t *trk, uint32_t raw_rl)
{
	uint32_t needed;

	if (raw_rl == trk->candidate) {
		if (trk->candidate_count < CANDIDATE_COUNT_MAX) {
			trk->candidate_count++;
		}
	} else {
		trk->candidate = raw_rl;
		trk->candidate_count = 1u;
	}
	needed = (trk->candidate == LINE_RL_CENTRED)
		 ? LINE_LOST_SAMPLES : LINE_CONFIRM_SAMPLES;
	if (trk->candidate_count >= needed) {
		trk->filtered = trk->candidate;
	}
	return trk->filtered;
}

/* J_n GP7-black samples running = one event; then nothing until GP7 has
   been off the crossbar LINE_JUNCTION_REARM_SAMPLES running. */
static bool junction_step(line_track_t *trk, bool mark_black)
{
	bool event = false;

	if (mark_black) {
		trk->clear_run = 0u;
		if (trk->both_black_run < CANDIDATE_COUNT_MAX) {
			trk->both_black_run++;
		}
		if (trk->junction_armed &&
		    (trk->both_black_run >= LINE_JUNCTION_SAMPLES)) {
			trk->junction_armed = false;
			event = true;
		}
	} else {
		trk->both_black_run = 0u;
		if (trk->clear_run < CANDIDATE_COUNT_MAX) {
			trk->clear_run++;
		}
		if (trk->clear_run >= LINE_JUNCTION_REARM_SAMPLES) {
			trk->junction_armed = true;
		}
	}
	return event;
}

static uint32_t rail_run(uint32_t run, uint16_t raw)
{
	if (!ir_at_rail(raw)) {
		return 0u;
	}
	return (run < CANDIDATE_COUNT_MAX) ? run + 1u : run;
}

void line_track_step(line_track_t *trk, uint16_t left_raw, uint16_t right_raw,
		     bool adc_ok, bool barcode_hold, bool mark_black,
		     line_track_out_t *out)
{
	line_state_fields_t *f = &out->fields;
	uint32_t raw_rl;
	uint32_t rl;
	int32_t error;
	bool fault;

	trk->left_rail_run = rail_run(trk->left_rail_run, left_raw);
	trk->right_rail_run = rail_run(trk->right_rail_run, right_raw);
	fault = !adc_ok ||
		(trk->left_rail_run >= IR_RAIL_FAULT_SAMPLES) ||
		(trk->right_rail_run >= IR_RAIL_FAULT_SAMPLES);

	if (adc_ok) {
		trk->left_black = ir_classify(&trk->left_cal, left_raw,
					      trk->left_black);
		trk->right_black = ir_classify(&trk->right_cal, right_raw,
					       trk->right_black);
		out->left_norm = ir_normalise(&trk->left_cal, left_raw);
		out->right_norm = ir_normalise(&trk->right_cal, right_raw);
	} else {
		out->left_norm = 0u;
		out->right_norm = 0u;
	}
	raw_rl = (trk->right_black ? LINE_RL_RIGHT_DEV : 0u) |
		 (trk->left_black ? LINE_RL_LEFT_DEV : 0u);
	rl = debounce(trk, raw_rl);

	/* GP7 only: rl 11 is the centred car, not a junction.  Not held by a
	   barcode read, because the crossbar under GP7 starts one itself; no
	   barcode bar is J_n long. */
	out->junction = junction_step(trk, mark_black);
	out->crossing = trk->both_black_run >= LINE_JUNCTION_SAMPLES;

	/* noise-filtered: only a confirmed (debounced) black, or a barcode
	   under the car, counts as seeing the line */
	if (((rl != LINE_RL_CENTRED) || barcode_hold) && !fault) {
		trk->dark_ms = 0u;
	} else if (trk->dark_ms < (LINE_ST_DARK_MASK * LINE_ST_DARK_UNIT_MS)) {
		trk->dark_ms++;
	} else {
		/* saturated */
	}

	error = ((int32_t)out->right_norm - (int32_t)out->left_norm) / 10;
	if (trk->steer_invert) {
		error = -error;
	}
	/* rl 11 keeps its error: centred, both inside the line, the small
	   difference still steers */
	if (barcode_hold || fault) {
		rl = barcode_hold ? LINE_RL_CENTRED : rl;
		error = 0;
	}

	trk->counter++;
	f->rl = rl;
	f->steer = line_state_steer_for(rl, trk->steer_invert);
	f->barcode = barcode_hold;
	f->lost = fault || (trk->dark_ms >= LINE_LOST_MS);
	f->fault = fault;
	f->cal = trk->calibrated;
	f->counter = trk->counter;
	f->error = error;
	f->dark_ms = trk->dark_ms;
	out->word = line_state_pack(f);
}
