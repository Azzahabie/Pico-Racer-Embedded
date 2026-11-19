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
 *
 * A one-letter Code 39 read (start, letter, stop) is 29 widths.  With
 * BARCODE_END_ON_STOP 1 the capture decodes them the moment the stop
 * character's last bar ends and, if they are a valid *X* (either way
 * round), ENDs the read on the next sample instead of after the quiet
 * zone, so a crossbar close behind the sheet cannot join the read.  If
 * the last bar runs on into a crossbar printed against it, the read is
 * tried with that bar taken as narrow, and ENDs the same way.
 *
 * Every sample first goes through a glitch filter: a GP7 run of black or
 * white BARCODE_GLITCH_TICKS samples or shorter is merged into the runs
 * either side, so a dropout of a few ms inside the crossbar neither opens
 * nor splits a read.  Longer runs are seen that many samples late at both
 * ends, so their widths are kept.
 *
 * Inside an open read, a black run far longer than the read's widths so
 * far is not a bar (BARCODE_NOT_BAR_RATIO_X10): the junction side may fire
 * on it, and when it does the read ENDs there (bar_capture_junction()).
 */

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"

/* Widths in a one-letter Code 39 read: 3 symbols of 9, 2 gaps. */
#define BAR_ONE_LETTER_WIDTHS (29u)

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
	BAR_ABORT_CANCELLED,	/* mission state changed mid-read           */
	BAR_ABORT_JUNCTION	/* the black is a junction's crossbar       */
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
	uint32_t first[BAR_ONE_LETTER_WIDTHS];	/* this read's first widths */
	bool end_due;		/* complete: END on the next sample        */
	bool complete;		/* this read ended as a valid *X*          */
	char letter;		/* its letter, when complete               */
	bool seen;		/* GP7 after the glitch filter            */
	uint8_t pending;	/* samples GP7 has read the other way      */
	uint16_t widest;	/* widest width of this read so far       */
	uint16_t widest_before;	/* the same, before the last width        */
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

/* Bars of a read are arriving: reading, past its first black run and
   short of the 29 widths of a one-letter barcode.  The first run is not
   counted, because a crossbar opens a read of its own (and every Code 39
   barcode starts with a narrow bar).  Nor is anything after the 29th
   width of a read that did not end there: that barcode is over (it was
   not a valid *X*), so a long black after it is the crossbar.  Nor is a
   black run too long to be a bar of this read (BARCODE_NOT_BAR_RATIO_X10,
   line_config.h): a crossbar that starts inside a read that is not a
   barcode (a speck, a dropout, an unreadable sheet).  A read that is
   complete counts until its END (the sample after), so the junction side
   sees the END before any junction can fall due. */
bool bar_capture_in_read(const bar_capture_t *cap);

/* GP7 after the glitch filter: what the capture saw this sample.  The
   junction side counts its black runs on this too, so both see the same
   runs (LINE_JUNCTION_RULE 1). */
bool bar_capture_black(const bar_capture_t *cap);

/* A junction fired while this read is open: GP7's black run is the
   crossbar.  A read the crossbar opened itself (no width yet) is dropped:
   BAR_EV_ABORT, BAR_ABORT_JUNCTION.  A read with widths before the
   crossbar ENDs: BAR_EV_END, so barcode_task judges those widths (and
   prints them if malformed); the crossbar is not one of them, and the
   rest of it opens no new read (the capture waits for a quiet zone, as
   after an abort).  A complete read due to END is left to END, and
   nothing happens when no read is open: BAR_EV_NONE. */
bar_event_t bar_capture_junction(bar_capture_t *cap);

/* Build the 4-byte message for an event. */
void bar_msg_pack(uint8_t msg[BAR_MSG_SIZE], bar_event_t event,
		  uint8_t read_id, uint16_t value);

const char *bar_abort_name(bar_abort_t reason);

#endif
