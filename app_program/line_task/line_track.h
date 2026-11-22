#ifndef ROBOCAR_LINE_TRACK_H
#define ROBOCAR_LINE_TRACK_H

/*
 * line_track.h - one 1 ms step of line tracking: classification, the
 * 0bRL debounce, junction detection (J_n samples, one event, rearm),
 * noise-filtered line loss, the position error and stuck-sensor detection.
 *
 * Pure logic, no kernel calls, so tests/line checks it on a PC.
 * ir_sense_task feeds it the two ADC readings and publishes the result.
 */

#include <stdbool.h>
#include <stdint.h>

#include "ir_cal.h"
#include "line_state.h"

typedef struct {
	ir_cal_sensor_t left_cal;	/* GP26 */
	ir_cal_sensor_t right_cal;	/* GP27 */
	bool calibrated;
	bool steer_invert;

	bool left_black;		/* hysteresis state per sensor   */
	bool right_black;
	uint32_t filtered;		/* debounced rl                  */
	uint32_t candidate;
	uint32_t candidate_count;

	uint32_t both_black_run;	/* junction debounce: GP7 black run */
	uint32_t clear_run;		/* GP7 white samples since leaving it */
	bool junction_armed;
	bool junction_due;		/* fires once the gate's wait ends */
	bool on_crossbar;		/* fired, GP7 still on that black */
	bool run_turning;
	uint16_t pair_run;	/* LINE_JUNCTION_PAIR: both line sensors black */
	uint32_t pair_mm;	/* travel with both black */
	uint32_t l_mm;		/* travel with the left sensor black (debug) */
	uint32_t r_mm;		/* travel with the right sensor black (debug) */
	uint32_t dbg_pair_mm;	/* a run just ended: its length, mm (debug) */
	uint32_t dbg_l_mm;
	uint32_t dbg_r_mm;
	bool dbg_turning;
	uint16_t gp_ago;	/* samples since the barcode sensor was black */
	bool pair_fired;	/* this both-black run has fired */		/* this GP7 black run began while
					   the scan gate was shut */

	uint32_t dark_ms;		/* since the last confirmed black */
	uint32_t left_rail_run;
	uint32_t right_rail_run;
	uint16_t left_hold;	/* crack filter: ms a white left still reads black */
	uint16_t right_hold;
	uint16_t left_nhold;	/* the last black reading, held */
	uint16_t right_nhold;
	uint32_t counter;
} line_track_t;

typedef struct {
	uint32_t word;		/* line_state, ready to publish        */
	line_state_fields_t fields;
	bool junction;		/* one junction event, this sample only */
	bool crossing;		/* still on that junction's crossbar    */
	uint16_t left_norm;	/* 0 white .. 1000 black               */
	uint16_t right_norm;
} line_track_out_t;

/*
 * What the barcode side tells the junction detector, every sample
 * (LINE_JUNCTION_RULE 1, from junction_arm_step()):
 *   in_read   bars of an open barcode read are arriving: GP7's black run
 *             is a bar, never a junction, unless it is too long to be a
 *             bar of that read (bar_capture_in_read());
 *   armed_due a complete barcode armed the junction and GP7's black run
 *             has confirmed it (LINE_ARMED_CONFIRM_MM): fire now;
 *   wait      a complete read is waiting for barcode_task's verdict: a
 *             junction that is due waits (at most
 *             LINE_ARMED_VERDICT_WAIT_MS), so nav finds the command;
 *   turning   the scan gate is shut (LINE_SCAN_GATE 1, scan_gate.h): a GP7
 *             black run that begins now never fires by the length rule,
 *             however long it lasts.  armed_due still fires.
 * Outside a read, GP7 black LINE_JUNCTION_SAMPLES running still fires (the
 * length rule, the fallback when no barcode armed it).
 */
typedef struct {
	bool in_read;
	bool armed_due;
	bool wait;
	bool turning;
	bool armed;	/* a barcode has armed the next junction (pair rule) */
	uint32_t moved_mm;	/* travel this sample (pair rule) */
	bool manoeuvre;		/* motion turning/bypassing/searching, or just
				   after a bypass: no junction at all */
} line_junction_gate_t;

/* Start from the configured thresholds, uncalibrated, junction armed. */
void line_track_init(line_track_t *trk, const ir_cal_sensor_t *left,
		     const ir_cal_sensor_t *right, bool steer_invert);

/* A passed calibration replaces both sensors' levels and thresholds; from
   then on each sample adapts them (ir_cal_adapt). */
void line_track_set_cal(line_track_t *trk, const ir_cal_sensor_t *left,
			const ir_cal_sensor_t *right);

/* Forget junction history (mission change): a junction the car is sitting
   on does not fire again; it rearms once the sensors are seen off it.  The
   line, debounce and loss state are kept, so a resume is not "lost". */
void line_track_disarm_junction(line_track_t *trk);

/*
 * One sample.  adc_ok false = the ADC read failed (counts as a fault).
 * barcode_hold true = a barcode is under the car: rl/steer/error held at 0;
 * the dark time and the lost bit still follow the real sensors.
 * mark_black = the barcode sensor (GP7) reads black (with a gate, after
 * the capture's glitch filter, bar_capture_black()); a long enough run of
 * it is the junction (LINE_JUNCTION_SAMPLES).  rl 11 is never a junction:
 * both sensors are inside the line whenever the car is centred.
 */
void line_track_step(line_track_t *trk, uint16_t left_raw, uint16_t right_raw,
		     bool adc_ok, bool barcode_hold, bool mark_black,
		     line_track_out_t *out);

/* The same with the barcode side's gate (above).  gate NULL is
   line_track_step(): the length rule alone, LINE_JUNCTION_RULE 0. */
void line_track_step_gate(line_track_t *trk, uint16_t left_raw,
			  uint16_t right_raw, bool adc_ok, bool barcode_hold,
			  bool mark_black, const line_junction_gate_t *gate,
			  line_track_out_t *out);

/* Crack filter (22 Nov 2026, car: the jigsaw seams between the track
   tiles cross the tape unevenly, so one sensor reads a white crack while
   the other is still on black, which reads as "the line drifted": a kick
   at every seam).  A sensor that was black and reads white for less than
   LINE_CRACK_MS (1 ms samples) still reports black with its last black
   reading.  The levels' adaptation still sees the real reading.  0 = off. */
#ifndef LINE_CRACK_MS
#define LINE_CRACK_MS	(0u)
#endif
#endif
