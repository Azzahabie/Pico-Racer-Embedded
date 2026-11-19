#ifndef ROBOCAR_JUNCTION_ARM_H
#define ROBOCAR_JUNCTION_ARM_H

/*
 * junction_arm.h - the barcode-armed junction (LINE_JUNCTION_RULE 1, the
 * team's rule, 19 Nov): once GP7 has read a complete, valid barcode and
 * barcode_task has stored its command, the next black run GP7 sees is the
 * junction, whatever its length.  With no barcode armed, the length rule
 * (LINE_JUNCTION_SAMPLES) is the fallback.
 *
 * The order is what makes it work: the read ends as soon as its stop
 * character is in (bar_capture), barcode_task stores the command, and only
 * then is the junction armed, so nav always finds the command when the
 * junction flag arrives.
 *
 *   IDLE  --complete read sent--> WAIT --command stored--> ARMED
 *   WAIT  --rejected, or no verdict in LINE_ARMED_VERDICT_WAIT_MS--> IDLE
 *         (back to ARMED if an older barcode had armed it)
 *   ARMED --junction (either rule)--> IDLE
 *   ARMED --LINE_ARMED_HOLD_MM of travel with no junction--> IDLE
 *   any   --mission change--> IDLE
 *
 * Pure logic, no kernel calls, so tests/line checks it on a PC.
 * ir_sense_task calls junction_arm_step() once a sample, after the barcode
 * capture and before line_track_step_gate(), which applies the gate.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"
#include "line_track.h"

typedef enum {
	JARM_IDLE = 0,	/* no barcode: the length rule alone              */
	JARM_WAIT,	/* complete read sent: barcode_task's verdict due  */
	JARM_ARMED	/* command stored: the next black is the junction  */
} jarm_phase_t;

/* What happened in one step, for ir_sense_task's console. */
typedef enum {
	JARM_EV_NONE = 0,
	JARM_EV_ARMED,		/* command stored: armed (or armed again)     */
	JARM_EV_REJECTED,	/* barcode_task stored no command             */
	JARM_EV_NO_VERDICT,	/* no verdict in LINE_ARMED_VERDICT_WAIT_MS    */
	JARM_EV_DISARM_DISTANCE,	/* LINE_ARMED_HOLD_MM, no junction     */
	JARM_EV_DISARM_MISSION	/* mission changed                            */
} jarm_event_t;

typedef struct {
	jarm_phase_t phase;
	bool was_armed;		/* WAIT: an older barcode had armed it       */
	uint8_t read_id;	/* WAIT: the read whose verdict is awaited   */
	char letter;		/* the letter GP7's capture decoded           */
	uint32_t seq_before;	/* WAIT: command sequence when it ended       */
	uint32_t waited;	/* WAIT: samples so far                       */
	uint32_t cmd;		/* ARMED: the command word it armed with      */
	uint32_t travel_mm;	/* ARMED: travel since armed                  */
	uint32_t black_ms;	/* GP7's current black run: samples          */
	uint32_t black_mm;	/* and travel                                 */
} junction_arm_t;

void junction_arm_init(junction_arm_t *arm);

/*
 * A read ended complete and valid (bar_capture_t.complete on its END).
 * cmd_before is barcode_cmd_word() read BEFORE that END was sent:
 * barcode_task, on the other core, may store the new command at once.
 */
void junction_arm_read_done(junction_arm_t *arm, uint8_t read_id, char letter,
			    uint32_t cmd_before);

/*
 * One sample.  judged = barcode_task's judged word (barcode_rx.h), cmd =
 * barcode_cmd_word(), loaded in that order (barcode_task stores the
 * command first), black = GP7 after the capture's glitch filter
 * (bar_capture_black()), moved_mm = travel since the last call,
 * in_read = bar_capture_in_read() (bars of an open read are arriving).
 * Fills the gate for line_track_step_gate() and returns what to print.
 */
jarm_event_t junction_arm_step(junction_arm_t *arm, uint32_t judged,
			       uint32_t cmd, bool black, uint32_t moved_mm,
			       bool in_read, line_junction_gate_t *gate);

/* A junction fired (either rule): the arming is used up.  Returns true if
   it was armed, so the junction was the armed one. */
bool junction_arm_used(junction_arm_t *arm);

/* Mission change: forget any barcode.  Returns JARM_EV_DISARM_MISSION if
   it was armed or waiting, else JARM_EV_NONE. */
jarm_event_t junction_arm_cancel(junction_arm_t *arm);

#endif
