#ifndef ROBOCAR_BARCODE_RX_H
#define ROBOCAR_BARCODE_RX_H

/*
 * barcode_rx.h - barcode_task's side of bar_mbf: assemble one read's widths
 * from ir_sense_task's messages, then decode and check it against the
 * letter -> action table.
 *
 * Pure logic, no kernel calls, so tests/line checks it on a PC.
 */

#include <stdbool.h>
#include <stdint.h>

#include "bar_capture.h"
#include "barcode_action.h"
#include "barcode_decode.h"

typedef enum {
	BARCODE_RX_NONE = 0,	/* nothing finished yet                     */
	BARCODE_RX_READ,	/* a read ended: evaluate it                */
	BARCODE_RX_ABORTED	/* a read was abandoned: reject it          */
} barcode_rx_event_t;

typedef struct {
	bool active;		/* widths of read_id are being collected */
	bool overflow;		/* more widths than fit                   */
	uint8_t read_id;
	uint16_t count;
	uint32_t widths[BARCODE_W_MAX];
	bar_abort_t abort_reason;
} barcode_rx_t;

typedef enum {
	BARCODE_REJECT_NONE = 0,
	BARCODE_REJECT_DECODE,		/* malformed: see decode error     */
	BARCODE_REJECT_NOT_ONE_LETTER,	/* decoded, but not one letter     */
	BARCODE_REJECT_UNKNOWN,		/* letter not in the loaded table  */
	BARCODE_REJECT_ABORTED,		/* capture abandoned the read      */
	BARCODE_REJECT_OVERFLOW		/* widths lost on the way          */
} barcode_reject_t;

typedef struct {
	bool accepted;
	barcode_reject_t reject;
	barcode_decode_result_t decode;
	char letter;			/* valid when accepted            */
	barcode_action_t action;	/* valid when accepted            */
} barcode_verdict_t;

void barcode_rx_init(barcode_rx_t *rx);

/* No read is being collected: a safe point to apply a map update. */
bool barcode_rx_idle(const barcode_rx_t *rx);

/* Drop anything being collected (mission change). */
void barcode_rx_discard(barcode_rx_t *rx);

/* Feed one 4-byte bar_mbf message.  A message from a new read_id drops
   whatever an older read left behind. */
barcode_rx_event_t barcode_rx_feed(barcode_rx_t *rx,
				   const uint8_t msg[BAR_MSG_SIZE]);

/* Decide on the read that barcode_rx_feed() just finished, then make the
   assembler idle again. */
void barcode_rx_evaluate(barcode_rx_t *rx, barcode_rx_event_t event,
			 const barcode_table_t *table,
			 barcode_verdict_t *verdict);

const char *barcode_reject_name(barcode_reject_t reject);

#endif
