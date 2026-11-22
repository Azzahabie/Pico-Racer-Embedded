/*
 * line_track.c - one 1 ms step of line tracking.
 */

#include <stddef.h>

#include "line_track.h"

#define CANDIDATE_COUNT_MAX (1000u)

/* both_black_run stops counting at CANDIDATE_COUNT_MAX: a longer J_n would
   never fire */
#if (LINE_JUNCTION_SAMPLES < 1u) || (LINE_JUNCTION_SAMPLES > CANDIDATE_COUNT_MAX)
#error "LINE_JUNCTION_SAMPLES must be 1..1000"
#endif

void line_track_disarm_junction(line_track_t *trk)
{
	trk->both_black_run = 0u;
	trk->clear_run = 0u;
	trk->junction_armed = false;
	trk->junction_due = false;
	trk->on_crossbar = false;
	trk->run_turning = false;
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
	trk->junction_due = false;
	trk->on_crossbar = false;
	trk->run_turning = false;
	trk->pair_run = 0u;
	trk->pair_mm = 0u;
	trk->gp_ago = 60000u;
	trk->pair_fired = true;	/* fires only after the pair has left black */
	trk->dark_ms = LINE_ST_DARK_MASK * LINE_ST_DARK_UNIT_MS;
	trk->left_rail_run = 0u;
	trk->right_rail_run = 0u;
	trk->left_hold = 0u;
	trk->right_hold = 0u;
	trk->left_nhold = 0u;
	trk->right_nhold = 0u;
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
   been off the crossbar LINE_JUNCTION_REARM_SAMPLES running.  With a gate
   (LINE_JUNCTION_RULE 1): never on a bar of an open barcode read (a run
   too long to be one is not), an armed junction fires once confirmed, and
   a due junction waits for the verdict of a complete read.  A run that
   began while the scan gate was shut (turning) never fires by length.
   With none, in_read, armed_due, wait and turning are false and this is
   the length rule alone. */
/* LINE_JUNCTION_PAIR (straddle layout, line_config.h): a junction is both
   line sensors black LINE_PAIR_SAMPLES running, with the barcode sensor
   (about 2 cm ahead) black within LINE_PAIR_GP_WINDOW samples.  In a curve
   the barcode sensor alone can sit on the line 450 ms (car 13:16:44: a
   false length-rule junction), but the line only ever covers one line
   sensor there; a crossbar covers both.  Barcode-armed junctions too, so a
   curve after a barcode cannot take the turn. */
#ifndef LINE_JUNCTION_PAIR
#define LINE_JUNCTION_PAIR	0
#endif
#ifndef LINE_PAIR_SAMPLES
#define LINE_PAIR_SAMPLES	40u
#endif
#ifndef LINE_PAIR_MM10
#define LINE_PAIR_MM10		175u	/* 17.5 mm: crossbar 25 mm, widest bar 15 mm */
#endif
#ifndef LINE_PAIR_GP_WINDOW
#define LINE_PAIR_GP_WINDOW	1500u
#endif

static bool junction_step(line_track_t *trk, bool mark_black,
			  bool pair_black, const line_junction_gate_t *gate)
{
	bool in_read = (gate != NULL) && gate->in_read;
	bool armed_due = (gate != NULL) && gate->armed_due;
	bool wait = (gate != NULL) && gate->wait;
	bool event = false;

	if (mark_black) {
		if (trk->both_black_run == 0u) {
			/* the run's first sample: the scan gate now holds for
			   all of it */
			trk->run_turning = (gate != NULL) && gate->turning;
		}
		trk->clear_run = 0u;
		if (trk->both_black_run < CANDIDATE_COUNT_MAX) {
			trk->both_black_run++;
		}
#if !LINE_JUNCTION_PAIR
		if (trk->junction_armed && !in_read &&
		    (((trk->both_black_run >= LINE_JUNCTION_SAMPLES) &&
		      !trk->run_turning) || armed_due)) {
			trk->junction_armed = false;
			trk->junction_due = true;
		}
#else
		(void)armed_due;
#endif
	} else {
		trk->both_black_run = 0u;
		trk->on_crossbar = false;
		trk->run_turning = false;
		if (trk->clear_run < CANDIDATE_COUNT_MAX) {
			trk->clear_run++;
		}
		/* not while a junction is held: it has not fired yet */
		if ((trk->clear_run >= LINE_JUNCTION_REARM_SAMPLES) &&
		    !trk->junction_due) {
			trk->junction_armed = true;
		}
	}
#if LINE_JUNCTION_PAIR
	if (mark_black) {
		trk->gp_ago = 0u;
	} else if (trk->gp_ago < 60000u) {
		trk->gp_ago++;
	} else {
		/* long ago */
	}
	/* debug: each black run's length on the pair and each sensor */
	if (gate != NULL) {
		bool lb = trk->left_black;
		bool rb = trk->right_black;

		if (lb) {
			trk->l_mm += gate->moved_mm;
		} else if (trk->l_mm > 0u) {
			trk->dbg_l_mm = trk->l_mm;
			trk->l_mm = 0u;
		} else {
			/* white */
		}
		if (rb) {
			trk->r_mm += gate->moved_mm;
		} else if (trk->r_mm > 0u) {
			trk->dbg_r_mm = trk->r_mm;
			trk->r_mm = 0u;
		} else {
			/* white */
		}
		if (!pair_black && (trk->pair_mm > 0u)) {
			trk->dbg_pair_mm = trk->pair_mm;
		}
		trk->dbg_turning = gate->turning;
	}
	if (pair_black) {
		if (trk->pair_run < 60000u) {
			trk->pair_run++;
		}
		if ((gate != NULL) && (trk->pair_mm < 100000u)) {
			trk->pair_mm += gate->moved_mm;
		}
	} else {
		trk->pair_run = 0u;
		trk->pair_mm = 0u;
		trk->pair_fired = false;
	}
	/* not held by a barcode read: the crossbar reaches the barcode sensor
	   (about 2 cm ahead) first and opens a read there, still open when the
	   line pair reaches it (car 13:36: Z armed, the junction missed).
	   Told from a barcode bar by length instead: both black LINE_PAIR_MM
	   of travel, longer than any bar */
	(void)in_read;
	/* after a barcode (armed): the next both black is the junction, at
	   once (the barcode's own bars are behind it: armed after its
	   decode).  Without one: both black LINE_PAIR_MM10 while straight. */
	if (!trk->pair_fired &&
	    !((gate != NULL) && gate->manoeuvre) &&
	    (((gate != NULL) && gate->armed && (trk->pair_run >= LINE_PAIR_SAMPLES)) ||
	     ((trk->pair_run >= LINE_PAIR_SAMPLES) &&
	      ((trk->pair_mm * 10u) >= LINE_PAIR_MM10) &&
	      (trk->gp_ago <= LINE_PAIR_GP_WINDOW) &&
	      !((gate != NULL) && gate->turning)))) {
		/* only while going straight (the scan gate's turn limit, and
		   shut while motion turns, searches or bypasses) */
		trk->pair_fired = true;
		trk->junction_due = true;
	}
#else
	(void)pair_black;
#endif
	if (trk->junction_due && !wait) {
		trk->junction_due = false;
		trk->on_crossbar = mark_black;
		event = true;
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
	line_track_step_gate(trk, left_raw, right_raw, adc_ok, barcode_hold,
			     mark_black, NULL, out);
}

void line_track_step_gate(line_track_t *trk, uint16_t left_raw,
			  uint16_t right_raw, bool adc_ok, bool barcode_hold,
			  bool mark_black, const line_junction_gate_t *gate,
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
		/* the levels follow the readings, but only once calibrated, and
		   not while a fault or a barcode makes the reading suspect */
		if (trk->calibrated && !fault && !barcode_hold) {
			ir_cal_adapt(&trk->left_cal, left_raw, trk->left_black);
			ir_cal_adapt(&trk->right_cal, right_raw,
				     trk->right_black);
		}
		out->left_norm = ir_normalise(&trk->left_cal, left_raw);
		out->right_norm = ir_normalise(&trk->right_cal, right_raw);
	} else {
		out->left_norm = 0u;
		out->right_norm = 0u;
	}
	{
		bool l_blk = trk->left_black;
		bool r_blk = trk->right_black;
#if LINE_CRACK_MS > 0u
		if (trk->left_black) {
			trk->left_hold = (uint16_t)LINE_CRACK_MS;
			trk->left_nhold = out->left_norm;
		} else if (trk->left_hold > 0u) {
			trk->left_hold--;
			l_blk = true;			/* a crack, not the edge */
			out->left_norm = trk->left_nhold;
		} else {
			/* white for longer: really off */
		}
		if (trk->right_black) {
			trk->right_hold = (uint16_t)LINE_CRACK_MS;
			trk->right_nhold = out->right_norm;
		} else if (trk->right_hold > 0u) {
			trk->right_hold--;
			r_blk = true;
			out->right_norm = trk->right_nhold;
		} else {
			/* white for longer: really off */
		}
#endif
		raw_rl = (r_blk ? LINE_RL_RIGHT_DEV : 0u) |
			 (l_blk ? LINE_RL_LEFT_DEV : 0u);
	}
	rl = debounce(trk, raw_rl);

	/* GP7 only: rl 11 is the centred car, not a junction.  Not held by a
	   barcode read's start, because the crossbar under GP7 starts one
	   itself; with a gate, held while the bars of a read arrive, and a
	   run that began while turning is not a crossbar. */
	out->junction = junction_step(trk, mark_black,
				      trk->left_black && trk->right_black, gate);
	out->crossing = trk->on_crossbar ||
			(((gate == NULL) || !gate->in_read) && !trk->run_turning &&
			 (trk->both_black_run >= LINE_JUNCTION_SAMPLES));

	/* noise-filtered: only a confirmed (debounced) black counts as seeing
	   the line.  Not a barcode hold: it blanks rl below, but the dark time
	   and the lost bit follow the real sensors, or motion reads "line
	   seen" with both sensors on white and never loses the line */
	if ((rl != LINE_RL_CENTRED) && !fault) {
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
#ifndef LINE_BARCODE_HOLD_STEER
#define LINE_BARCODE_HOLD_STEER	1	/* TEST: 0 = bit 4 only, steering live */
#endif
	if ((barcode_hold && (LINE_BARCODE_HOLD_STEER != 0)) || fault) {
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
