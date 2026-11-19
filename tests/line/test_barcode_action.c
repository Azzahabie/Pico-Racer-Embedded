/* Host test for barcode_action.c: the table, the 7-byte map update and the
   barcode-command word.  Built twice: as committed (no compiled-in table)
   and with the writeup's example compiled in (-DBARCODE_DEFAULT_TABLE_SET=1,
   A left, B right, C straight, D U-turn; tests/line/Makefile). */

#include <string.h>

#include "check.h"
#include "../../app_program/common/messages.h"
#include "../../app_program/line_task/barcode_action.h"
#include "../host/vectors.h"	/* map_example_*: packed by the laptop's rules */

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
	char text[BARCODE_TABLE_TEXT_SIZE];
	char letter;

	barcode_table_init(&table);
	barcode_table_to_string(&table, text);
#if BARCODE_DEFAULT_TABLE_SET
	/* the compiled-in table is loaded at init, map version 0 */
	CHECK(table.loaded);
	CHECK(table.version == 0u);
	CHECK(strcmp(text, BARCODE_DEFAULT_TABLE) == 0);
	CHECK(barcode_table_lookup(&table, 'A', &action) && (action == BARCODE_ACT_LEFT));
	CHECK(barcode_table_lookup(&table, 'B', &action) && (action == BARCODE_ACT_RIGHT));
	CHECK(barcode_table_lookup(&table, 'C', &action) && (action == BARCODE_ACT_FORWARD));
	CHECK(barcode_table_lookup(&table, 'D', &action) && (action == BARCODE_ACT_UTURN));
	CHECK(barcode_table_lookup(&table, 'Z', &action) && (action == BARCODE_ACT_FORWARD));
	CHECK(!barcode_table_lookup(&table, '5', &action));	/* Code 39 digit */
	(void)letter;
#else
	CHECK(!table.loaded);	/* no approved default table: none compiled in */
	CHECK(strcmp(text, "-") == 0);
	for (letter = 'A'; letter <= 'Z'; ++letter) {
		CHECK(!barcode_table_lookup(&table, letter, &action));
	}
#endif
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
	barcode_table_t fresh;
	barcode_action_t action;

	barcode_table_init(&table);
	barcode_table_init(&fresh);
	CHECK(!barcode_table_from_string(&table, ""));
	CHECK(!barcode_table_from_string(&table, "FLRUFLRUFLRUFLRUFLRUFLRUF"));	/* 25 */
	CHECK(!barcode_table_from_string(&table, "FLRUFLRUFLRUFLRUFLRUFLRUFLX"));	/* 27 */
	CHECK(!barcode_table_from_string(&table, "FLRUFLRUFLRUFLRUFLRUFLRUFX"));	/* bad char */
	CHECK(!barcode_table_from_string(&table, "flrufLRUFLRUFLRUFLRUFLRUFL"));	/* lower case */
	CHECK(!barcode_table_from_string(&table, "SLRUFLRUFLRUFLRUFLRUFLRUFL"));	/* S, not F */
	CHECK(memcmp(&table, &fresh, sizeof(table)) == 0);	/* unchanged */
#if !BARCODE_DEFAULT_TABLE_SET
	CHECK(!table.loaded);
#endif
	CHECK(barcode_table_from_string(&table, "UUUUUUUUUUUUUUUUUUUUUUUUUL"));
	CHECK(barcode_table_lookup(&table, 'A', &action) && (action == BARCODE_ACT_UTURN));
	CHECK(barcode_table_lookup(&table, 'Z', &action) && (action == BARCODE_ACT_LEFT));
}

/* Every action, from a map, for every letter: the map holds no "unknown"
   value, so once one is loaded each of A-Z gives one of the four. */
static void test_every_action_from_map(void)
{
	static const char *const names[4] = { "FORWARD", "LEFT", "RIGHT", "U_TURN" };
	barcode_table_t table;
	uint8_t actions[BARCODE_LETTER_COUNT];
	uint8_t map[BARCODE_MAP_SIZE];
	barcode_action_t action;
	uint32_t a;
	uint32_t i;
	uint32_t word;

	for (a = 0u; a < 4u; ++a) {
		(void)memset(actions, (int)a, sizeof(actions));
		make_map(map, a + 1u, actions);
		barcode_table_init(&table);
		CHECK(barcode_table_apply_map(&table, map, BARCODE_MAP_SIZE));
		CHECK(table.version == (a + 1u));
		for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
			action = (barcode_action_t)7;
			CHECK(barcode_table_lookup(&table, (char)('A' + i), &action));
			CHECK((uint32_t)action == a);
			/* what barcode_task stores and nav reads at the junction */
			word = barcode_cmd_pack(i, (uint32_t)action, i + 1u);
			CHECK(barcode_cmd_letter(word) == i);
			CHECK(barcode_cmd_action(word) == a);
			CHECK(barcode_cmd_pending(word, 0u));
		}
		CHECK(strcmp(barcode_action_name(a), names[a]) == 0);
	}
}

/* The writeup's example (A left, B right, C straight, D U-turn, the rest
   straight) as the 7 bytes the laptop sends: mv 1 in bits 0-3, then 2 bits
   a letter, A first, so A=1 at bits 4-5, B=2 at 6-7, C=0, D=3 at 10-11:
   0x0C91 (C, by hand), little end first. */
static void test_writeup_example_bytes(void)
{
	static const uint8_t map[BARCODE_MAP_SIZE] = {
		0x91u, 0x0Cu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u
	};
	barcode_table_t table;
	barcode_action_t action;
	char text[BARCODE_TABLE_TEXT_SIZE];
	char letter;

	barcode_table_init(&table);
	CHECK(barcode_table_apply_map(&table, map, BARCODE_MAP_SIZE));
	CHECK(table.version == 1u);
	barcode_table_to_string(&table, text);
	CHECK(strcmp(text, "LRFUFFFFFFFFFFFFFFFFFFFFFF") == 0);
	CHECK(barcode_table_lookup(&table, 'A', &action) && (action == BARCODE_ACT_LEFT));
	CHECK(barcode_table_lookup(&table, 'B', &action) && (action == BARCODE_ACT_RIGHT));
	CHECK(barcode_table_lookup(&table, 'C', &action) && (action == BARCODE_ACT_FORWARD));
	CHECK(barcode_table_lookup(&table, 'D', &action) && (action == BARCODE_ACT_UTURN));
	for (letter = 'E'; letter <= 'Z'; ++letter) {
		CHECK(barcode_table_lookup(&table, letter, &action) &&
		      (action == BARCODE_ACT_FORWARD));
	}
}

/* The comms vector (tests/host/vectors.h, generated from packed_size.py,
   which tools/telemetry_decode.py's pack_map matches in test_decoder.py):
   barcode_task reads the same table the laptop meant. */
static void test_comms_vector_map(void)
{
	barcode_table_t table;
	barcode_action_t action;
	uint32_t i;

	barcode_table_init(&table);
	CHECK(barcode_table_apply_map(&table, map_example_bytes,
				      (int)sizeof(map_example_bytes)));
	CHECK(table.version == map_example_mv);
	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		CHECK(barcode_table_lookup(&table, (char)('A' + i), &action) &&
		      ((uint32_t)action == map_example_act[i]));
	}
}

/* barcode_table_to_string gives back what barcode_table_from_string took. */
static void test_table_text(void)
{
	static const char text_in[] = "UFLRRLFUUFLRRLFUUFLRRLFUUF";
	barcode_table_t table;
	barcode_table_t again;
	char text[BARCODE_TABLE_TEXT_SIZE];

	barcode_table_init(&table);
	CHECK(barcode_table_from_string(&table, text_in));
	barcode_table_to_string(&table, text);
	CHECK(strcmp(text, text_in) == 0);
	barcode_table_init(&again);
	CHECK(barcode_table_from_string(&again, text));
	CHECK(memcmp(again.action, table.action, sizeof(table.action)) == 0);
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
	test_every_action_from_map();
	test_writeup_example_bytes();
	test_comms_vector_map();
	test_table_text();
	test_cmd_word();
	return check_report(BARCODE_DEFAULT_TABLE_SET
			    ? "barcode table (compiled-in " BARCODE_DEFAULT_TABLE ")"
			    : "barcode table");
}
