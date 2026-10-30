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

	uint32_t dark_ms;		/* since the last confirmed black */
	uint32_t left_rail_run;
	uint32_t right_rail_run;
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

/* Start from the configured thresholds, uncalibrated, junction armed. */
void line_track_init(line_track_t *trk, const ir_cal_sensor_t *left,
		     const ir_cal_sensor_t *right, bool steer_invert);

/* A passed calibration replaces both sensors' thresholds. */
void line_track_set_cal(line_track_t *trk, const ir_cal_sensor_t *left,
			const ir_cal_sensor_t *right);

/* Forget junction history (mission change): a junction the car is sitting
   on does not fire again; it rearms once the sensors are seen off it.  The
   line, debounce and loss state are kept, so a resume is not "lost". */
void line_track_disarm_junction(line_track_t *trk);

/*
 * One sample.  adc_ok false = the ADC read failed (counts as a fault).
 * barcode_hold true = a barcode is under the car: rl/steer/error held at 0.
 * mark_black = the barcode sensor (GP7) reads black; a long enough run of
 * it is the junction (LINE_JUNCTION_SAMPLES).  rl 11 is never a junction:
 * both sensors are inside the line whenever the car is centred.
 */
void line_track_step(line_track_t *trk, uint16_t left_raw, uint16_t right_raw,
		     bool adc_ok, bool barcode_hold, bool mark_black,
		     line_track_out_t *out);

#endif
