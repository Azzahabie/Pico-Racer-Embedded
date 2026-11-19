#ifndef COMMS_FOLD_H
#define COMMS_FOLD_H

/* comms_task's fold: one valid event body into the S cache (MQTT_SCHEMA.md
 * section 4).  Fields are stored as on the wire.  faults, mpv, drp, tlt
 * and trn are not folded from events: comms_task pulls them before each
 * S, and mis too (its folded copy is overwritten).
 *
 * Plain C, no kernel calls, so it builds on a PC too
 * (tests/host/test_messages.c). */

#include <stdbool.h>
#include <stdint.h>

#include "../common/messages.h"

/* What the fold keeps between events besides S itself. */
typedef struct {
	bool		have_dist;	/* a motion event has been folded */
	uint16_t	dist_at_change;	/* dist_mm when mot last changed */
} T_COMMS_FOLD;

/* Fold one valid event (from comm_w_unpack_body) into s. */
static inline void comms_fold(T_COMM_W_S *s, T_COMMS_FOLD *f,
			      const T_COMM_W_BODY *b)
{
	uint32_t	cm;

	switch(b->kind) {
	case COMM_W_KIND_MOTION:
		/* dst is cm since the last motion change, derived from the
		   wrapping dist_mm counter (handles one wrap), clamped to 8 bits. */
		if(!f->have_dist || b->motion_state != s->mot) {
			f->dist_at_change = b->dist_mm;
			f->have_dist = true;
		}
		s->mot = b->motion_state;
		cm = comm_w_dist_delta_mm(b->dist_mm, f->dist_at_change) / 10U;
		s->dst = (uint8_t)((cm > 255U) ? 255U : cm);
		break;
	case COMM_W_KIND_LINE:
		s->lin = b->line_state;
		s->dev = b->deviation;
		break;
	case COMM_W_KIND_BARCODE:
		/* Only an accepted read changes S.  barcode_task sends every
		   rejected read as letter 31, action 0, and the crossbar at a
		   junction is one, so copying it wiped the letter at every
		   junction.  The E record still logs each reject. */
		if(b->letter != COMM_W_BARCODE_UNREADABLE) {
			s->bar = b->letter;
			s->act = b->action;
		}
		break;
	case COMM_W_KIND_IMU:
		s->hum = b->hump_count;
		s->hgt = b->hump_height_mm;
		break;
	case COMM_W_KIND_OBSTACLE:
		s->obs = b->width_cm;
		s->byp = b->bypass;
		break;
	default:	/* COMM_W_KIND_MISSION */
		s->mis = b->mission_state;
		break;
	}
}

#endif /* COMMS_FOLD_H */
