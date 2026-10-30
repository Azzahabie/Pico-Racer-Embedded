/*
 * bar_capture.c - barcode capture, one call per 1 ms sample.
 */

#include "bar_capture.h"

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
}

bool bar_capture_busy(const bar_capture_t *cap)
{
	return cap->phase != BAR_IDLE;
}

bool bar_capture_reading(const bar_capture_t *cap)
{
	return cap->phase == BAR_ACTIVE;
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

bar_event_t bar_capture_step(bar_capture_t *cap, bool black, uint16_t *width)
{
	switch (cap->phase) {
	case BAR_IDLE:
		if (black) {
			cap->phase = BAR_ACTIVE;
			cap->level = true;
			cap->run = 1u;
			cap->count = 0u;
			cap->narrowest = UINT16_MAX;
			cap->ticks = 1u;
			cap->read_id++;
			cap->reason = BAR_ABORT_NONE;
		}
		return BAR_EV_NONE;

	case BAR_DISCARD:
		/* the rest of the abandoned barcode must not start a new read */
		if (black) {
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

	if (++cap->ticks > BARCODE_READ_MAX_TICKS) {
		return abort_read(cap, BAR_ABORT_TIMEOUT);
	}
	if (black == cap->level) {
		if (cap->run < UINT16_MAX) {
			cap->run++;
		}
		if (!black && (cap->run >= quiet_ticks(cap))) {
			cap->phase = BAR_IDLE;	/* trailing white is not a width */
			return BAR_EV_END;
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
	cap->count++;
	if (cap->run < cap->narrowest) {
		cap->narrowest = cap->run;
	}
	cap->level = black;
	cap->run = 1u;
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
	case BAR_ABORT_NONE:
	default:
		return "none";
	}
}
