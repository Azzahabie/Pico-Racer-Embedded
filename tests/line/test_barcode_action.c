/* Host test for barcode_action.c: the table, the 7-byte map update and the
   barcode-command word. */

#include <string.h>

#include "check.h"
#include "../../app_program/common/messages.h"
#include "../../app_program/line_task/barcode_action.h"

/* Build a map the way the publisher does: mv, then 2 bits per letter. */
static void make_map(uint8_t map[BARCODE_MAP_SIZE], uint32_t version,
		     const uint8_t actions[BARCODE_LETTER_COUNT])
{
	uint32_t pos = 0u;
	uint32_t i;

	(void)memset(map, 0, BARCODE_MAP_SIZE);
	comm_w_put_bits(map, &pos, version, 4u);
	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		comm_w_put_bits(map, &pos, actions[i], 2u);
	}
}

static void test_empty_table(void)
{
	barcode_table_t table;
	barcode_action_t action = BARCODE_ACT_UTURN;
	char letter;

	barcode_table_init(&table);
	CHECK(!table.loaded);	/* no approved default table: none compiled in */
	for (letter = 'A'; letter <= 'Z'; ++letter) {
		CHECK(!barcode_table_lookup(&table, letter, &action));
	}
}

static void test_map_update(void)
{
	barcode_table_t table;
	barcode_table_t before;
	uint8_t actions[BARCODE_LETTER_COUNT];
	uint8_t map[BARCODE_MAP_SIZE];
	barcode_action_t action;
	uint32_t i;

	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		actions[i] = (uint8_t)(i % 4u);	/* all four actions appear */
	}
	make_map(map, 9u, actions);
	barcode_table_init(&table);
	CHECK(barcode_table_apply_map(&table, map, BARCODE_MAP_SIZE));
	CHECK(table.loaded);
	CHECK(table.version == 9u);
	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		CHECK(barcode_table_lookup(&table, (char)('A' + i), &action));
		CHECK((uint32_t)action == (i % 4u));
	}
	CHECK(barcode_table_lookup(&table, 'D', &action) && (action == BARCODE_ACT_UTURN));
	CHECK(barcode_table_lookup(&table, 'C', &action) && (action == BARCODE_ACT_RIGHT));
	CHECK(barcode_table_lookup(&table, 'B', &action) && (action == BARCODE_ACT_LEFT));
	CHECK(barcode_table_lookup(&table, 'A', &action) && (action == BARCODE_ACT_FORWARD));
	CHECK(barcode_table_lookup(&table, 'Z', &action) && (action == BARCODE_ACT_LEFT));
	CHECK(!barcode_table_lookup(&table, '*', &action));

	/* idempotent */
	before = table;
	CHECK(barcode_table_apply_map(&table, map, BARCODE_MAP_SIZE));
	CHECK(memcmp(&before, &table, sizeof(table)) == 0);

	/* an invalid update keeps the working table */
	CHECK(!barcode_table_apply_map(&table, map, BARCODE_MAP_SIZE - 1));
	CHECK(!barcode_table_apply_map(&table, map, BARCODE_MAP_SIZE + 1));
	CHECK(!barcode_table_apply_map(&table, NULL, BARCODE_MAP_SIZE));
	CHECK(memcmp(&before, &table, sizeof(table)) == 0);
}

static void test_string_table(void)
{
	barcode_table_t table;
	barcode_action_t action;

	barcode_table_init(&table);
	CHECK(!barcode_table_from_string(&table, ""));
	CHECK(!barcode_table_from_string(&table, "FLRUFLRUFLRUFLRUFLRUFLRUF"));	/* 25 */
	CHECK(!barcode_table_from_string(&table, "FLRUFLRUFLRUFLRUFLRUFLRUFLX"));	/* 27 */
	CHECK(!barcode_table_from_string(&table, "FLRUFLRUFLRUFLRUFLRUFLRUFX"));	/* bad char */
	CHECK(!table.loaded);
	CHECK(barcode_table_from_string(&table, "UUUUUUUUUUUUUUUUUUUUUUUUUL"));
	CHECK(barcode_table_lookup(&table, 'A', &action) && (action == BARCODE_ACT_UTURN));
	CHECK(barcode_table_lookup(&table, 'Z', &action) && (action == BARCODE_ACT_LEFT));
}

static void test_cmd_word(void)
{
	uint32_t word;

	word = barcode_cmd_pack(25u, BARCODE_ACT_UTURN, 0x1FFu);
	CHECK(barcode_cmd_letter(word) == 25u);
	CHECK(barcode_cmd_action(word) == BARCODE_ACT_UTURN);
	CHECK(barcode_cmd_seq(word) == 0xFFu);
	CHECK(barcode_cmd_pending(word, 0u));
	CHECK(!barcode_cmd_pending(word, 0xFFu));		/* already taken */
	CHECK(!barcode_cmd_pending(BARCODE_CMD_NONE, 7u));	/* nothing stored */
	word = barcode_cmd_pack(BARCODE_LETTER_NONE, 0u, 3u);	/* cleared */
	CHECK(!barcode_cmd_pending(word, 2u));
	CHECK(strcmp(barcode_action_name(BARCODE_ACT_UTURN), "U_TURN") == 0);
	CHECK(strcmp(barcode_action_name(7u), "?") == 0);
	CHECK(barcode_letter_index('A') == 0u);
	CHECK(barcode_letter_index('z') == BARCODE_LETTER_NONE);
}

int main(void)
{
	test_empty_table();
	test_map_update();
	test_string_table();
	test_cmd_word();
	return check_report("barcode table");
}
