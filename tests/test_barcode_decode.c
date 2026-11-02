#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../app_program/line_task/barcode_decode.h"

#define SYMBOL_WIDTHS 9u
#define TRIALS_PER_CASE 200u
#define MAX_TEST_WIDTHS BARCODE_MAX_ELEMENTS

static const char patterns[27][SYMBOL_WIDTHS + 1u] = {
	"WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",
	"WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",
	"NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",
	"WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",
	"NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",
	"WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",
	"WWNNWNNNN", "NWWNWNNNN", "NWNNWNWNN"
};

static uint32_t random_state = 0x7A39D51Bu;

static uint32_t random_next(void)
{
	random_state ^= random_state << 13;
	random_state ^= random_state >> 17;
	random_state ^= random_state << 5;
	return random_state;
}

static void append_symbol(const char *pattern, uint32_t narrow,
			  uint32_t wide, uint32_t *widths,
			  uint16_t *position)
{
	uint16_t i;

	for (i = 0u; i < SYMBOL_WIDTHS; ++i) {
		widths[(*position)++] =
			(pattern[i] == 'W') ? wide : narrow;
	}
}

static int pattern_index(char symbol)
{
	if (symbol == '*') {
		return 26;
	}
	if ((symbol >= 'A') && (symbol <= 'Z')) {
		return (int)(symbol - 'A');
	}
	return -1;
}

static int synthesize(const char *text, uint32_t ratio_x100,
		      int32_t bias_percent, int reverse, uint32_t *output,
		      uint16_t *output_count)
{
	uint32_t widths[MAX_TEST_WIDTHS];
	uint16_t position = 0u;
	uint16_t symbol_number;
	uint16_t text_length = (uint16_t)strlen(text);
	uint32_t narrow = 80u + (random_next() % 101u);
	uint32_t wide = (narrow * ratio_x100 + 50u) / 100u;
	char symbol;
	int index;

	for (symbol_number = 0u; symbol_number < text_length + 2u;
	     ++symbol_number) {
		if (symbol_number == 0u || symbol_number == text_length + 1u) {
			symbol = '*';
		} else {
			symbol = text[symbol_number - 1u];
		}
		index = pattern_index(symbol);
		if (index < 0) {
			return 0;
		}
		append_symbol(patterns[index], narrow, wide, widths, &position);
		if (symbol_number + 1u < text_length + 2u) {
			widths[position++] = narrow;
		}
	}

	for (symbol_number = 0u; symbol_number < position; ++symbol_number) {
		uint32_t base = widths[symbol_number];
		int32_t jitter = (int32_t)(random_next() % 11u) - 5;
		int32_t edge_offset =
			((symbol_number & 1u) == 0u)
				? bias_percent : -bias_percent;
		int32_t jittered =
			(int32_t)((base * (uint32_t)(100 + jitter) + 50u) / 100u);
		int32_t measured =
			jittered + (edge_offset * (int32_t)narrow) / 100;

		widths[symbol_number] = (uint32_t)measured;
	}

	if (reverse) {
		for (symbol_number = 0u; symbol_number < position; ++symbol_number) {
			output[symbol_number] = widths[position - 1u - symbol_number];
		}
	} else {
		(void)memcpy(output, widths, (size_t)position * sizeof(widths[0]));
	}
	*output_count = position;
	return 1;
}

static int run_cases(const char *text)
{
	static const uint32_t ratios[] = { 250u, 300u };
	static const uint32_t biases[] = { 10u, 15u, 20u };
	uint32_t widths[MAX_TEST_WIDTHS];
	barcode_decode_result_t result;
	uint16_t width_count;
	uint32_t ratio_index;
	uint32_t direction;
	uint32_t trial;

	for (ratio_index = 0u;
	     ratio_index < sizeof(ratios) / sizeof(ratios[0]);
	     ++ratio_index) {
		for (direction = 0u; direction < 2u; ++direction) {
			for (trial = 0u; trial < TRIALS_PER_CASE; ++trial) {
				int32_t bias = (int32_t)biases[trial %
					(sizeof(biases) / sizeof(biases[0]))];

				if ((random_next() & 1u) != 0u) {
					bias = -bias;
				}
				if (!synthesize(text, ratios[ratio_index],
						bias, (int)direction,
						widths, &width_count)) {
					(void)fprintf(stderr,
						"%s trial %u: synthesis failed\n",
						text, (unsigned)trial);
					return 0;
				}
				barcode_decode_widths(widths, width_count, &result);
				if (!result.valid || (result.reversed != (direction != 0u)) ||
				    (strcmp(result.text, text) != 0)) {
					(void)fprintf(stderr,
						"%s ratio=%u bias=%d%% dir=%s trial=%u: error=%s, decoded=\"%s\"\n",
						text, (unsigned)ratios[ratio_index],
						bias, direction ? "rev" : "fwd",
						(unsigned)trial,
						barcode_decode_error_name(result.error),
						result.text);
					return 0;
				}
			}
		}
	}
	return 1;
}

int main(void)
{
	static const char *const test_strings[] = {
		"HELLO",
		"ABCDEFGHIJKLMNOPQRSTUVWX",
		"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
	};
	char letter[2] = { 'A', '\0' };
	size_t i;

	/* The demo case: one letter per barcode, every letter, both ways. */
	for (letter[0] = 'A'; letter[0] <= 'Z'; ++letter[0]) {
		if (!run_cases(letter)) {
			return EXIT_FAILURE;
		}
	}
	for (i = 0u; i < sizeof(test_strings) / sizeof(test_strings[0]); ++i) {
		if (!run_cases(test_strings[i])) {
			return EXIT_FAILURE;
		}
	}
	(void)puts("barcode decoder: PASS (A-Z singly and 3 strings, "
		   "fwd + rev, 800 trials each)");
	return EXIT_SUCCESS;
}
