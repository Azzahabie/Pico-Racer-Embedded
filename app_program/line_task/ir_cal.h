#ifndef ROBOCAR_IR_CAL_H
#define ROBOCAR_IR_CAL_H

/*
 * ir_cal.h - per-sensor calibration, classification and normalisation for
 * the analogue line sensors (GP26, GP27).
 *
 * Pure logic, no kernel calls, so tests/line checks it on a PC.  Internally
 * every reading is turned into "darkness" (raw when black reads high,
 * 4095 - raw when it reads low), so one set of rules serves both polarities.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"

#define IR_ADC_MAX      (4095u)
#define IR_NORM_BLACK   (1000u)		/* normalised: 0 white .. 1000 black */

typedef enum {
	IR_CAL_OK = 0,
	IR_CAL_NO_SAMPLES,
	IR_CAL_RAIL_STUCK,	/* reading pinned at an ADC rail       */
	IR_CAL_LOW_CONTRAST	/* white too close to the black level  */
} ir_cal_result_t;

/* One sensor's calibration.  Levels and thresholds are in darkness units. */
typedef struct {
	bool black_is_high;
	uint16_t white;		/* reference white                    */
	uint16_t black;		/* reference black (> white)          */
	uint16_t rise;		/* white -> black at or above this    */
	uint16_t fall;		/* black -> white at or below this    */
} ir_cal_sensor_t;

/* Samples recorded during a calibration window. */
typedef struct {
	uint16_t min_dark;
	uint16_t max_dark;
	uint32_t samples;
	uint32_t rail_samples;
	bool black_is_high;
} ir_cal_acc_t;

/* The configured (tested) thresholds, before any calibration has run. */
ir_cal_sensor_t ir_cal_default(bool black_is_high, uint16_t rise_raw,
			       uint16_t fall_raw);

void ir_cal_begin(ir_cal_acc_t *acc, bool black_is_high);
void ir_cal_add(ir_cal_acc_t *acc, uint16_t raw);

/* Ends the window.  On IR_CAL_OK *out holds the new calibration; on any
   failure *out is left unchanged. */
ir_cal_result_t ir_cal_finish(const ir_cal_acc_t *acc,
			      const ir_cal_sensor_t *defaults,
			      ir_cal_sensor_t *out);

const char *ir_cal_result_name(ir_cal_result_t result);

/* true when raw sits at an ADC rail (disconnected or shorted sensor). */
bool ir_at_rail(uint16_t raw);

/* Black or white, with hysteresis around rise/fall. */
bool ir_classify(const ir_cal_sensor_t *cal, uint16_t raw, bool was_black);

/* 0 (white reference) .. 1000 (black reference), clamped. */
uint16_t ir_normalise(const ir_cal_sensor_t *cal, uint16_t raw);

#endif
