#ifndef ROBOCAR_BAR_CAPTURE_H
#define ROBOCAR_BAR_CAPTURE_H

/*
 * bar_capture.h - barcode capture in ir_sense_task: one call per 1 ms
 * sample of the GP7 digital sensor, widths counted in sampling ticks.
 *
 * Pure logic, no kernel calls, so tests/line checks it on a PC.  Widths
 * are tick counts, never timestamps, so there is no timer wraparound.
 *
 * Every read gets a new read_id, carried on each message to barcode_task,
 * so widths left over from an aborted or partly delivered read can never
 * join the next one.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"

typedef enum {
	BAR_EV_NONE = 0,
	BAR_EV_WIDTH,	/* one bar or space finished: width in ticks       */
	BAR_EV_END,	/* quiet zone Q seen after the last bar            */
	BAR_EV_ABORT	/* read abandoned: reason in bar_capture_t.reason  */
} bar_event_t;

typedef enum {
	BAR_ABORT_NONE = 0,
	BAR_ABORT_TOO_MANY,	/* more than W_max widths                   */
	BAR_ABORT_TOO_LONG,	/* one width above W_long                   */
	BAR_ABORT_TIMEOUT,	/* whole read above BARCODE_READ_MAX_TICKS  */
	BAR_ABORT_OVERFLOW,	/* the message buffer to barcode_task full  */
	BAR_ABORT_CANCELLED	/* mission state changed mid-read           */
} bar_abort_t;

typedef enum {
	BAR_IDLE = 0,	/* waiting for the first black                     */
	BAR_ACTIVE,	/* timing bars and spaces                          */
	BAR_DISCARD	/* after an abort: wait for a quiet zone first     */
} bar_phase_t;

typedef struct {
	bar_phase_t phase;
	bool level;		/* black = true                           */
	uint16_t run;		/* ticks at the current level             */
	uint16_t count;		/* widths sent in this read               */
	uint16_t narrowest;
	uint32_t ticks;		/* ticks since the read started           */
	uint8_t read_id;
	bar_abort_t reason;	/* of the last abort                      */
} bar_capture_t;

/* Message to barcode_task: 4 bytes on bar_mbf. */
#define BAR_MSG_SIZE (4)
#define BAR_MSG_WIDTH ((uint8_t)BAR_EV_WIDTH)
#define BAR_MSG_END   ((uint8_t)BAR_EV_END)
#define BAR_MSG_ABORT ((uint8_t)BAR_EV_ABORT)

void bar_capture_init(bar_capture_t *cap);

/* One sample.  On BAR_EV_WIDTH, *width is the finished width in ticks. */
bar_event_t bar_capture_step(bar_capture_t *cap, bool black, uint16_t *width);

/* Abandon a read in progress (mission change, or a message that could not
   be sent).  Returns BAR_EV_ABORT if a read was in progress, else NONE. */
bar_event_t bar_capture_abort(bar_capture_t *cap, bar_abort_t reason);

/* A read is in progress (or being discarded). */
bool bar_capture_busy(const bar_capture_t *cap);

/* A read is in progress, not counting the discard after an abort: GP7 can
   sit on black indefinitely after width_too_long, and that must not hold
   line_state or silence the console. */
bool bar_capture_reading(const bar_capture_t *cap);

/* Build the 4-byte message for an event. */
void bar_msg_pack(uint8_t msg[BAR_MSG_SIZE], bar_event_t event,
		  uint8_t read_id, uint16_t value);

const char *bar_abort_name(bar_abort_t reason);

#endif
