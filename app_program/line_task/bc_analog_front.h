#ifndef ROBOCAR_BC_ANALOG_FRONT_H
#define ROBOCAR_BC_ANALOG_FRONT_H

/*
 * bc_analog_front.h - the analog barcode sensor's front end
 * (BARCODE_SENSOR_ANALOG 1, line_config.h).  Ours, not the other team's.
 *
 * Every 1 ms sample of the IR module's AO (GP28, ADC2, darker = higher
 * with BC_ANALOG_BLACK_HIGH 1) gives:
 *   norm     0 white .. 1000 black for the other team's capture (bc_analog_capture.h):
 *            the reading above the white floor, scaled by BC_ANALOG_SPAN_RAW;
 *   black    the floor-relative hysteresis of the lab test image (20 Nov):
 *            black from BC_ANALOG_ON_DELTA above the white floor, white
 *            again within BC_ANALOG_OFF_DELTA of it; the floor snaps down to
 *            any lighter reading and creeps up one count every
 *            BC_ANALOG_FLOOR_RISE_MS while white;
 *   seen     black after the glitch filter (BARCODE_GLITCH_TICKS), what
 *            the junction side counts (junction_arm.h, line_track.h).
 * and, from distance_state (motion_ds.h: dist_mm in whole mm, published
 * every 10 ms), the travel for each 1 ms sample:
 *   speed    the time between the last whole-mm changes of dist_mm: the
 *            mm gained over the newest changes no older than
 *            BC_FRONT_WIN_MS (at most BC_FRONT_EVENTS), divided by the time
 *            they took.  Once no change has come for longer than that
 *            mean interval, the speed is at most 1 mm over the time since
 *            the last change, so it falls smoothly when the car stops, and
 *            it is 0 after BC_FRONT_STOP_MS without one.  Reversing counts
 *            as travel (|change|).  Without distance_state (no motion task,
 *            or its counter stuck for LINE_SCAN_STALE_MS) the speed is
 *            BC_ANALOG_NO_DS_UM_PER_MS (a bench, 0 = none);
 *   step_um  speed x 1 ms, with the fraction carried to the next sample
 *            (bc_front_take_step()).
 *
 * Pure logic, no kernel calls, so tests/line checks it on a PC.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"

#define BC_FRONT_EVENTS		(8u)	/* whole-mm changes kept           */
#define BC_FRONT_WIN_MS		(250u)	/* newest changes used for speed   */
#define BC_FRONT_STOP_MS	(400u)	/* no change this long: speed 0    */
#define BC_FRONT_MAX_STEP_MM	(20)	/* a bigger jump in 10 ms: a reset */

typedef struct {
	/* the level */
	uint16_t dark;		/* last good reading, darker = higher      */
	uint16_t floor_v;	/* white floor, same units                 */
	uint16_t floor_ms;
	uint16_t norm;		/* 0..1000                                  */
	bool black;		/* floor-relative hysteresis               */
	bool seen;		/* after the glitch filter                 */
	uint32_t pending;	/* ms the level has read the other way     */
	/* the travel */
	bool ds_seen;		/* a distance_state word was taken         */
	bool ds_ok;		/* distance_state usable (last step)       */
	uint32_t ds_last;
	uint32_t ctr_ms;	/* ms since its counter last changed       */
	uint32_t t_ms;		/* ms since init (wraps after 49 days)     */
	uint32_t travel_mm;	/* |whole-mm changes| summed               */
	uint32_t ev_t[BC_FRONT_EVENTS];
	uint32_t ev_mm[BC_FRONT_EVENTS];
	uint8_t ev_n;
	uint8_t ev_head;	/* the newest change                        */
	uint32_t speed_x16;	/* um per ms x 16                           */
	uint32_t carry_x16;	/* fraction of a um carried                 */
} bc_front_t;

void bc_front_init(bc_front_t *f);

/*
 * One sample.  raw = the 12-bit ADC reading (above 4095, a failed read,
 * keeps the last good one), ds = motion_ds_read(), ds_up =
 * motor_task_flag() > 0, elapsed_ms = 1 + ticks missed.
 */
void bc_front_step(bc_front_t *f, uint16_t raw, uint32_t ds, bool ds_up,
		   uint32_t elapsed_ms);

/* The travel for one 1 ms sample, in um (call once per sample fed to the
   capture: 1 + ticks missed times). */
uint32_t bc_front_take_step(bc_front_t *f);

/* The speed in mm/s, for the console. */
uint32_t bc_front_speed_mm_s(const bc_front_t *f);

#endif
