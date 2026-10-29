#ifndef ROBOCAR_BARCODE_ACTION_H
#define ROBOCAR_BARCODE_ACTION_H

/*
 * barcode_action.h - the barcode letter -> junction movement table, the
 * live map update (MQTT_SCHEMA.md 8.3), and the barcode-command word that
 * barcode_task hands to nav_ctrl_task.
 *
 * Pure logic, no kernel calls, so tests/line checks it on a PC.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"

/* The 2-bit action code (MQTT_SCHEMA.md section 12 item 7, owned by
   Buddy 3).  The same values go in the E/S barcode records and the map. */
typedef enum {
	BARCODE_ACT_FORWARD = 0,	/* straight on */
	BARCODE_ACT_LEFT    = 1,
	BARCODE_ACT_RIGHT   = 2,
	BARCODE_ACT_UTURN   = 3
} barcode_action_t;

#define BARCODE_LETTER_COUNT (26u)
#define BARCODE_LETTER_NONE  (31u)	/* no / unreadable letter, as on the wire */
#define BARCODE_MAP_SIZE     (7)	/* 4-bit mv + 26 x 2-bit actions */

typedef struct {
	bool loaded;			/* false: every letter is unknown */
	uint8_t version;		/* mv of the map applied, 0-15    */
	uint8_t action[BARCODE_LETTER_COUNT];
} barcode_table_t;

/* Empty, or the compiled-in default if BARCODE_DEFAULT_TABLE_SET is 1. */
void barcode_table_init(barcode_table_t *table);

/* Parse a 26-character F/L/R/U string, 'A' first.  false (table unchanged)
   on any other length or character. */
bool barcode_table_from_string(barcode_table_t *table, const char *text);

/* Apply a 7-byte map.  false (table unchanged) unless length is exactly
   BARCODE_MAP_SIZE.  Every 2-bit value is a valid action. */
bool barcode_table_apply_map(barcode_table_t *table, const uint8_t *map,
			     int length);

/* Action for 'A'..'Z'.  false when the table is not loaded or the
   character is not a letter: the read is unknown, send no command. */
bool barcode_table_lookup(const barcode_table_t *table, char letter,
			  barcode_action_t *action);

/* Letter index 0 = A .. 25 = Z, or BARCODE_LETTER_NONE. */
uint32_t barcode_letter_index(char letter);

/* "FORWARD", "LEFT", "RIGHT", "U_TURN", or "?" for a value out of range. */
const char *barcode_action_name(uint32_t action);

/*
 * Barcode-command word, written by barcode_task only (one aligned store),
 * read by nav_ctrl_task at a junction:
 *   bits 0-4   letter index (BARCODE_LETTER_NONE = no command pending)
 *   bits 5-6   action
 *   bits 8-15  sequence, +1 on every change, so nav takes each command once
 */
#define BARCODE_CMD_LETTER_SHIFT (0u)
#define BARCODE_CMD_LETTER_MASK  (0x1Fu)
#define BARCODE_CMD_ACTION_SHIFT (5u)
#define BARCODE_CMD_ACTION_MASK  (0x03u)
#define BARCODE_CMD_SEQ_SHIFT    (8u)
#define BARCODE_CMD_SEQ_MASK     (0xFFu)

#define BARCODE_CMD_NONE ((uint32_t)BARCODE_LETTER_NONE)

uint32_t barcode_cmd_pack(uint32_t letter_index, uint32_t action,
			  uint32_t sequence);

static inline uint32_t barcode_cmd_letter(uint32_t word)
{
	return (word >> BARCODE_CMD_LETTER_SHIFT) & BARCODE_CMD_LETTER_MASK;
}

static inline uint32_t barcode_cmd_action(uint32_t word)
{
	return (word >> BARCODE_CMD_ACTION_SHIFT) & BARCODE_CMD_ACTION_MASK;
}

static inline uint32_t barcode_cmd_seq(uint32_t word)
{
	return (word >> BARCODE_CMD_SEQ_SHIFT) & BARCODE_CMD_SEQ_MASK;
}

/* A command is waiting that nav has not taken (taken_seq = last one used). */
static inline bool barcode_cmd_pending(uint32_t word, uint32_t taken_seq)
{
	return (barcode_cmd_letter(word) != BARCODE_LETTER_NONE) &&
	       (barcode_cmd_seq(word) != (taken_seq & BARCODE_CMD_SEQ_MASK));
}

#endif
