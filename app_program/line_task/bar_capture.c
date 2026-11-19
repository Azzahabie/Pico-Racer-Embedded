/*
 * bar_capture.c - barcode capture, one call per 1 ms sample.
 */

#include "bar_capture.h"
#include "barcode_decode.h"

#ifndef BARCODE_END_ON_STOP
#error "BARCODE_END_ON_STOP must come from line_config.h"
#endif
#ifndef BARCODE_GLITCH_TICKS
#error "BARCODE_GLITCH_TICKS must come from line_config.h"
#endif
#ifndef BARCODE_NOT_BAR_RATIO_X10
#error "BARCODE_NOT_BAR_RATIO_X10 must come from line_config.h"
#endif

void bar_capture_init(bar_capture_t *cap)
{
	cap->phase = BAR_IDLE;
	cap->level = false;
	cap->run = 0u;
	cap->count = 0u;
	cap->narrowest = UINT16_MAX;
	cap->ticks = 0u;
	cap->read_id = 0u;
	cap->reason = BAR_ABORT_NONE;
	cap->end_due = false;
	cap->complete = false;
	cap->letter = '\0';
	cap->seen = false;
	cap->pending = 0u;
	cap->widest = 0u;
	cap->widest_before = 0u;
	cap->gate_open = true;
	cap->ignored = 0u;
}

void bar_capture_set_gate(bar_capture_t *cap, bool open)
{
	cap->gate_open = open;
}

bool bar_capture_busy(const bar_capture_t *cap)
{
	return cap->phase != BAR_IDLE;
}

bool bar_capture_reading(const bar_capture_t *cap)
{
	return cap->phase == BAR_ACTIVE;
}

bool bar_capture_black(const bar_capture_t *cap)
{
	return cap->seen;
}

/* From any bar of a one-letter read, either way round, at most 8 narrow
   elements come before a wide one (C, from the Code 39 patterns), so from
   a read's 11th width on, its widths before the last space include a
   wide one, even in a read that started part way through a barcode. */
#define NOT_BAR_LATE_COUNT (10u)

/* The current black run is too long to be a bar of this read
   (line_config.h, BARCODE_NOT_BAR_RATIO_X10): longer than a ratio times
   the widest width before the space in front of it.  That space is left
   out because it may be the gap to the crossbar, any length up to the
   quiet zone.  Before the 11th width that widest width may be a narrow
   one and the run a wide bar, so the ratio is BARCODE_NOT_BAR_EARLY_X10;
   from the 11th width on it is BARCODE_NOT_BAR_RATIO_X10, and also at
   the 3rd width when the space before it was a dropout: under
   1 / BARCODE_DROPOUT_DIV of the bar before it, which no space of a
   barcode is. */
static bool not_a_bar(const bar_capture_t *cap)
{
	uint32_t ratio = BARCODE_NOT_BAR_RATIO_X10;

	if ((ratio == 0u) || !cap->level || (cap->count < 2u)) {
		return false;
	}
	if ((cap->count < NOT_BAR_LATE_COUNT) &&
	    ((cap->count != 2u) ||
	     ((cap->first[1] * BARCODE_DROPOUT_DIV) >= cap->first[0]))) {
		ratio = BARCODE_NOT_BAR_EARLY_X10;
	}
	return ((uint32_t)cap->run * 10u) > (ratio * (uint32_t)cap->widest_before);
}

bool bar_capture_in_read(const bar_capture_t *cap)
{
	return (cap->phase == BAR_ACTIVE) && (cap->count > 0u) &&
	       (cap->end_due ||
		((cap->count < BAR_ONE_LETTER_WIDTHS) && !not_a_bar(cap)));
}

/* The first 29 widths decode as one valid *X* (either way round). */
static bool decodes_one_letter(bar_capture_t *cap)
{
	barcode_decode_result_t res;

	barcode_decode_widths(cap->first, (uint16_t)BAR_ONE_LETTER_WIDTHS, &res);
	if (res.valid && (res.text[0] != '\0') && (res.text[1] == '\0')) {
		cap->end_due = true;
		cap->complete = true;
		cap->letter = res.text[0];
		return true;
	}
	return false;
}

/* The 29th width just ended (the stop character's last bar): a valid
   one-letter read ends now.  Anything else carries on as before. */
static void check_complete(bar_capture_t *cap)
{
	if ((BARCODE_END_ON_STOP != 0u) &&
	    (cap->count == BAR_ONE_LETTER_WIDTHS)) {
		(void)decodes_one_letter(cap);
	}
}

/* 28 widths in and the last bar runs on: a sheet printed right up to the
   crossbar, so its last bar and the crossbar are one black run.  Every
   Code 39 read ends with a narrow bar, like its 21st width (the first
   bar of the last symbol, narrow either way round), so once the run is
   twice that, take the last bar as that wide and try the read.  Valid:
   the barcode is over, it ENDs, and the rest of the run is the next
   black (the crossbar).  Returns true with *width the bar's width. */
static bool check_run_on(bar_capture_t *cap, uint16_t *width)
{
	uint32_t narrow;

	if ((BARCODE_END_ON_STOP == 0u) || !cap->level ||
	    (cap->count != (BAR_ONE_LETTER_WIDTHS - 1u))) {
		return false;
	}
	narrow = cap->first[BAR_ONE_LETTER_WIDTHS - 9u];
	if ((uint32_t)cap->run != (2u * narrow) + 1u) {
		return false;
	}
	cap->first[BAR_ONE_LETTER_WIDTHS - 1u] = narrow;
	if (!decodes_one_letter(cap)) {
		return false;
	}
	*width = (uint16_t)narrow;
	cap->count++;
	return true;
}

static uint32_t quiet_ticks(const bar_capture_t *cap)
{
	uint32_t q = BARCODE_Q_TICKS;
	uint32_t scaled;

	if (cap->narrowest != UINT16_MAX) {
		scaled = (uint32_t)cap->narrowest * BARCODE_Q_WIDTH_FACTOR;
		if (scaled > q) {
			q = scaled;
		}
	}
	return q;
}

static bar_event_t abort_read(bar_capture_t *cap, bar_abort_t reason)
{
	cap->phase = BAR_DISCARD;
	cap->reason = reason;
	cap->end_due = false;
	cap->complete = false;
	cap->run = 0u;		/* now counts white ticks towards a quiet zone */
	return BAR_EV_ABORT;
}

bar_event_t bar_capture_abort(bar_capture_t *cap, bar_abort_t reason)
{
	if (cap->phase != BAR_ACTIVE) {
		return BAR_EV_NONE;
	}
	return abort_read(cap, reason);
}

bar_event_t bar_capture_junction(bar_capture_t *cap)
{
	if ((cap->phase != BAR_ACTIVE) || cap->end_due) {
		return BAR_EV_NONE;
	}
	if (cap->count == 0u) {
		return abort_read(cap, BAR_ABORT_JUNCTION);
	}
	/* the widths before the crossbar go to barcode_task; wait for quiet */
	cap->phase = BAR_DISCARD;
	cap->run = 0u;
	return BAR_EV_END;
}

/* The glitch filter: the capture sees an edge only once GP7 has read the
   other way for more than BARCODE_GLITCH_TICKS samples running.  A
   shorter run counts as the level around it; a longer one is seen that
   many samples late at both ends, so its width is unchanged. */
static bool deglitch(bar_capture_t *cap, bool black)
{
	if (black == cap->seen) {
		cap->pending = 0u;
	} else if (++cap->pending > BARCODE_GLITCH_TICKS) {
		cap->seen = black;
		cap->pending = 0u;
	} else {
		/* not yet: still the level before */
	}
	return cap->seen;
}

bar_event_t bar_capture_step(bar_capture_t *cap, bool black, uint16_t *width)
{
	bool seen = deglitch(cap, black);

	switch (cap->phase) {
	case BAR_IDLE:
		if (seen && !cap->gate_open) {
			/* the scan gate: turning, so no read, and none on the
			   rest of this black (wait for quiet, as after an abort) */
			cap->phase = BAR_DISCARD;
			cap->run = 0u;
			cap->ignored++;
		} else if (seen) {
			cap->phase = BAR_ACTIVE;
			cap->level = true;
			cap->run = 1u;
			cap->count = 0u;
			cap->narrowest = UINT16_MAX;
			cap->ticks = 1u;
			cap->read_id++;
			cap->reason = BAR_ABORT_NONE;
			cap->end_due = false;
			cap->complete = false;
			cap->letter = '\0';
			cap->widest = 0u;
			cap->widest_before = 0u;
		}
		return BAR_EV_NONE;

	case BAR_DISCARD:
		/* the rest of the abandoned barcode must not start a new read */
		if (seen) {
			cap->run = 0u;
		} else if (++cap->run >= BARCODE_Q_TICKS) {
			cap->phase = BAR_IDLE;
		} else {
			/* still waiting for quiet */
		}
		return BAR_EV_NONE;

	case BAR_ACTIVE:
	default:
		break;
	}

	/* a complete *X*: no quiet zone needed (BARCODE_END_ON_STOP) */
	if (cap->end_due) {
		cap->end_due = false;
		cap->phase = BAR_IDLE;
		return BAR_EV_END;
	}
	if (++cap->ticks > BARCODE_READ_MAX_TICKS) {
		return abort_read(cap, BAR_ABORT_TIMEOUT);
	}
	if (seen == cap->level) {
		if (cap->run < UINT16_MAX) {
			cap->run++;
		}
		if (!seen && (cap->run >= quiet_ticks(cap))) {
			cap->phase = BAR_IDLE;	/* trailing white is not a width */
			return BAR_EV_END;
		}
		if (check_run_on(cap, width)) {
			return BAR_EV_WIDTH;	/* and END on the next sample */
		}
		if (cap->run > BARCODE_W_LONG_TICKS) {
			return abort_read(cap, BAR_ABORT_TOO_LONG);
		}
		return BAR_EV_NONE;
	}

	/* an edge: the previous bar or space is finished */
	if (cap->count >= BARCODE_W_MAX) {
		return abort_read(cap, BAR_ABORT_TOO_MANY);
	}
	*width = cap->run;
	if (cap->count < BAR_ONE_LETTER_WIDTHS) {
		cap->first[cap->count] = cap->run;
	}
	cap->count++;
	if (cap->run < cap->narrowest) {
		cap->narrowest = cap->run;
	}
	cap->widest_before = cap->widest;
	if (cap->run > cap->widest) {
		cap->widest = cap->run;
	}
	cap->level = seen;
	cap->run = 1u;
	check_complete(cap);
	return BAR_EV_WIDTH;
}

void bar_msg_pack(uint8_t msg[BAR_MSG_SIZE], bar_event_t event,
		  uint8_t read_id, uint16_t value)
{
	msg[0] = (uint8_t)event;
	msg[1] = read_id;
	msg[2] = (uint8_t)(value & 0xFFu);
	msg[3] = (uint8_t)(value >> 8);
}

const char *bar_abort_name(bar_abort_t reason)
{
	switch (reason) {
	case BAR_ABORT_TOO_MANY:
		return "too_many_widths";
	case BAR_ABORT_TOO_LONG:
		return "width_too_long";
	case BAR_ABORT_TIMEOUT:
		return "read_too_long";
	case BAR_ABORT_OVERFLOW:
		return "buffer_overflow";
	case BAR_ABORT_CANCELLED:
		return "cancelled";
	case BAR_ABORT_JUNCTION:
		return "junction";
	case BAR_ABORT_NONE:
	default:
		return "none";
	}
}
