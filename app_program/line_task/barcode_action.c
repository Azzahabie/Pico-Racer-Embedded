/*
 * barcode_action.c - letter -> junction movement table and map update.
 */

#include <stddef.h>

#include "../common/messages.h"	/* comm_w_get_bits: the wire bit order */
#include "barcode_action.h"

#define MAP_VERSION_BITS (4u)
#define MAP_ACTION_BITS  (2u)

_Static_assert((MAP_VERSION_BITS + (BARCODE_LETTER_COUNT * MAP_ACTION_BITS))
	       == (8u * BARCODE_MAP_SIZE),
	       "the map is exactly 7 bytes (MQTT_SCHEMA.md 8.3)");

#if BARCODE_DEFAULT_TABLE_SET
_Static_assert(sizeof(BARCODE_DEFAULT_TABLE) == (BARCODE_LETTER_COUNT + 1u),
	       "BARCODE_DEFAULT_TABLE needs one F/L/R/U per letter A-Z");
#endif

void barcode_table_init(barcode_table_t *table)
{
	uint32_t i;

	table->loaded = false;
	table->version = 0u;
	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		table->action[i] = (uint8_t)BARCODE_ACT_FORWARD;
	}
#if BARCODE_DEFAULT_TABLE_SET
	(void)barcode_table_from_string(table, BARCODE_DEFAULT_TABLE);
#endif
}

bool barcode_table_from_string(barcode_table_t *table, const char *text)
{
	uint8_t parsed[BARCODE_LETTER_COUNT];
	uint32_t i;

	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		switch (text[i]) {
		case 'F':
			parsed[i] = (uint8_t)BARCODE_ACT_FORWARD;
			break;
		case 'L':
			parsed[i] = (uint8_t)BARCODE_ACT_LEFT;
			break;
		case 'R':
			parsed[i] = (uint8_t)BARCODE_ACT_RIGHT;
			break;
		case 'U':
			parsed[i] = (uint8_t)BARCODE_ACT_UTURN;
			break;
		default:
			return false;	/* includes an early '\0' */
		}
	}
	if (text[BARCODE_LETTER_COUNT] != '\0') {
		return false;
	}
	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		table->action[i] = parsed[i];
	}
	table->loaded = true;
	return true;
}

bool barcode_table_apply_map(barcode_table_t *table, const uint8_t *map,
			     int length)
{
	uint32_t pos = 0u;
	uint32_t i;

	if ((map == NULL) || (length != BARCODE_MAP_SIZE)) {
		return false;
	}
	table->version = (uint8_t)comm_w_get_bits(map, &pos, MAP_VERSION_BITS);
	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		table->action[i] = (uint8_t)comm_w_get_bits(map, &pos,
							     MAP_ACTION_BITS);
	}
	table->loaded = true;
	return true;
}

uint32_t barcode_letter_index(char letter)
{
	if ((letter >= 'A') && (letter <= 'Z')) {
		return (uint32_t)(letter - 'A');
	}
	return BARCODE_LETTER_NONE;
}

bool barcode_table_lookup(const barcode_table_t *table, char letter,
			  barcode_action_t *action)
{
	uint32_t index = barcode_letter_index(letter);

	if (!table->loaded || (index == BARCODE_LETTER_NONE)) {
		return false;
	}
	*action = (barcode_action_t)table->action[index];
	return true;
}

const char *barcode_action_name(uint32_t action)
{
	switch (action) {
	case BARCODE_ACT_FORWARD:
		return "FORWARD";
	case BARCODE_ACT_LEFT:
		return "LEFT";
	case BARCODE_ACT_RIGHT:
		return "RIGHT";
	case BARCODE_ACT_UTURN:
		return "U_TURN";
	default:
		return "?";
	}
}

uint32_t barcode_cmd_pack(uint32_t letter_index, uint32_t action,
			  uint32_t sequence)
{
	return ((letter_index & BARCODE_CMD_LETTER_MASK)
		<< BARCODE_CMD_LETTER_SHIFT) |
	       ((action & BARCODE_CMD_ACTION_MASK)
		<< BARCODE_CMD_ACTION_SHIFT) |
	       ((sequence & BARCODE_CMD_SEQ_MASK) << BARCODE_CMD_SEQ_SHIFT);
}
