#ifndef ROBOCAR_SCAN_GATE_H
#define ROBOCAR_SCAN_GATE_H

/*
 * scan_gate.h - the scan gate (LINE_SCAN_GATE 1, the team's decision from
 * the prof's tip, 19 Nov: barcodes are always on straight segments, never
 * on curves).  A GP7 black may open a barcode read, or fire a junction by
 * the length rule, only if it starts while the car is going straight.
 *
 * The signal is the turn rate motion_task publishes in distance_state
 * (motion_ds.h: deg/s, counter-clockwise positive, averaged over 50 ms,
 * published every 10 ms).  Every 10 ms it goes through two low-pass
 * stages, each moving 1/LINE_SCAN_AVG_DIV of the way to its input, so the
 * car's weave on a straight averages out while a curve or a pivot stays.
 * The car is going straight while |average| < LINE_SCAN_MAX_TURN_DPS.
 *
 * Motion's own state (motion_state_read(), motor_task.h) comes first: in
 * a manoeuvre that is not line following (TURNING, BYPASS, LINE_SEARCH,
 * REVERSING: scan_gate_manoeuvre()) the gate is shut whatever the
 * average says, and when the manoeuvre ends both stages go back to 0, so
 * the straight after a junction pivot scans at once instead of waiting
 * about 2 s for the average to fall.  In every other state (LINE_FOLLOW,
 * and the stopped states IDLE, BRAKED, LINE_LOST, FAULT) the average
 * decides, as for a curve.
 *
 * Fails open: without distance_state (motion_up false, which ir_sense_task
 * takes from motor_task_flag() 0 as in a bench image, or the publish
 * counter unchanged for LINE_SCAN_STALE_MS) the gate is open whatever
 * the motion state, and the average starts again from 0.
 *
 * Pure logic, no kernel calls, so tests/line checks it on a PC.
 * ir_sense_task calls scan_gate_step() once a sample, before the barcode
 * capture (bar_capture_set_gate()) and the junction side
 * (line_junction_gate_t.turning).
 */

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"

/* The average is kept in deg/s x SCAN_GATE_SCALE. */
#define SCAN_GATE_SCALE		(1024)
/* One low-pass step every this many ms (motion publishes every 10 ms). */
#define SCAN_GATE_SAMPLE_MS	(10u)

typedef struct {
	int32_t stage1;		/* first low-pass stage, x SCAN_GATE_SCALE   */
	int32_t stage2;		/* the average, x SCAN_GATE_SCALE             */
	uint32_t sample_ms;	/* ms towards the next low-pass step          */
	uint32_t counter;	/* distance_state's counter, last seen        */
	uint32_t stale_ms;	/* ms since it last changed                   */
	bool available;		/* distance_state usable (last step)          */
	bool manoeuvre;		/* motion in a manoeuvre (last step): shut    */
	bool open;		/* a read may start (last step)               */
} scan_gate_t;

/* ds = motion_ds_read() now: its counter counts as fresh. */
void scan_gate_init(scan_gate_t *gate, uint32_t ds);

/*
 * One sample.  ds = motion_ds_read(), motion = motion_state_read() (a
 * motion_state_t, motion_sm.h), motion_up = motor_task_flag() > 0,
 * elapsed_ms = ms since the last call (1, more after missed ticks).
 * Returns true when a GP7 black may open a read: not in a manoeuvre and
 * going straight, or no distance_state (fails open).
 */
bool scan_gate_step(scan_gate_t *gate, uint32_t ds, uint32_t motion,
		    bool motion_up, uint32_t elapsed_ms);

/* true for the motion states that shut the gate on their own: TURNING,
   BYPASS, LINE_SEARCH and REVERSING.  false for the others and for a
   value that is not a state. */
bool scan_gate_manoeuvre(uint32_t motion);

/* The averaged turn rate in tenths of a deg/s, counter-clockwise (left)
   positive; 0 while distance_state is not available.  It keeps running
   during a manoeuvre and starts from 0 when the manoeuvre ends. */
int32_t scan_gate_avg_x10(const scan_gate_t *gate);

#endif
