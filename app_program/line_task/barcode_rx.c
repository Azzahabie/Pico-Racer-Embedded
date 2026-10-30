/*
 * barcode_rx.c - assemble and judge one barcode read.
 */

#include <string.h>

#include "barcode_rx.h"

void barcode_rx_init(barcode_rx_t *rx)
{
	rx->active = false;
	rx->overflow = false;
	rx->read_id = 0u;
	rx->count = 0u;
	rx->abort_reason = BAR_ABORT_NONE;
}

bool barcode_rx_idle(const barcode_rx_t *rx)
{
	return !rx->active;
}

void barcode_rx_discard(barcode_rx_t *rx)
{
	rx->active = false;
	rx->overflow = false;
	rx->count = 0u;
}

static void start_read(barcode_rx_t *rx, uint8_t read_id)
{
	rx->active = true;
	rx->overflow = false;
	rx->read_id = read_id;
	rx->count = 0u;
	rx->abort_reason = BAR_ABORT_NONE;
}

barcode_rx_event_t barcode_rx_feed(barcode_rx_t *rx,
				   const uint8_t msg[BAR_MSG_SIZE])
{
	uint8_t type = msg[0];
	uint8_t read_id = msg[1];
	uint16_t value = (uint16_t)(msg[2] | ((uint16_t)msg[3] << 8));

	if (!rx->active || (read_id != rx->read_id)) {
		/* a new read drops any stale partial.  An END or ABORT whose
		   widths never arrived still ends up as one rejected read:
		   END's width count will not match the none received. */
		start_read(rx, read_id);
	}

	switch (type) {
	case BAR_MSG_WIDTH:
		if (rx->count < BARCODE_W_MAX) {
			rx->widths[rx->count++] = value;
		} else {
			rx->overflow = true;
		}
		return BARCODE_RX_NONE;
	case BAR_MSG_END:
		/* the END carries the width count the capture sent */
		if (value != rx->count) {
			rx->overflow = true;
		}
		return BARCODE_RX_READ;
	case BAR_MSG_ABORT:
		rx->abort_reason = (bar_abort_t)value;
		return BARCODE_RX_ABORTED;
	default:
		barcode_rx_discard(rx);
		return BARCODE_RX_NONE;
	}
}

void barcode_rx_evaluate(barcode_rx_t *rx, barcode_rx_event_t event,
			 const barcode_table_t *table,
			 barcode_verdict_t *verdict)
{
	(void)memset(verdict, 0, sizeof(*verdict));
	verdict->letter = '\0';

	if (event == BARCODE_RX_ABORTED) {
		verdict->reject = BARCODE_REJECT_ABORTED;
	} else if (rx->overflow) {
		verdict->reject = BARCODE_REJECT_OVERFLOW;
	} else {
		barcode_decode_widths(rx->widths, rx->count, &verdict->decode);
		if (!verdict->decode.valid) {
			verdict->reject = BARCODE_REJECT_DECODE;
		} else if ((verdict->decode.text[0] == '\0') ||
			   (verdict->decode.text[1] != '\0')) {
			verdict->reject = BARCODE_REJECT_NOT_ONE_LETTER;
		} else if (!barcode_table_lookup(table, verdict->decode.text[0],
						 &verdict->action)) {
			verdict->reject = BARCODE_REJECT_UNKNOWN;
			verdict->letter = verdict->decode.text[0];
		} else {
			verdict->accepted = true;
			verdict->reject = BARCODE_REJECT_NONE;
			verdict->letter = verdict->decode.text[0];
		}
	}
	/* the widths stay readable for a diagnostic dump until the next read */
	rx->active = false;
}

const char *barcode_reject_name(barcode_reject_t reject)
{
	switch (reject) {
	case BARCODE_REJECT_DECODE:
		return "malformed";
	case BARCODE_REJECT_NOT_ONE_LETTER:
		return "not_one_letter";
	case BARCODE_REJECT_UNKNOWN:
		return "unknown_letter";
	case BARCODE_REJECT_ABORTED:
		return "aborted";
	case BARCODE_REJECT_OVERFLOW:
		return "widths_lost";
	case BARCODE_REJECT_NONE:
	default:
		return "none";
	}
}
