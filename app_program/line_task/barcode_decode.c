#include <stddef.h>
#include <string.h>

#include "barcode_decode.h"

#define BARCODE_PATTERN_ELEMENTS (9u)

static const char code39_patterns[27][BARCODE_PATTERN_ELEMENTS + 1u] = {
	"WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",
	"WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",
	"NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",
	"WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",
	"NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",
	"WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",
	"WWNNWNNNN", "NWWNWNNNN", "NWNNWNWNN"
};

static const char code39_characters[28] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ*";

static bool decode_symbol(const uint32_t *widths, bool reversed,
			  char *symbol, uint32_t *narrow_width)
{
	uint32_t sorted[BARCODE_PATTERN_ELEMENTS];
	char pattern[BARCODE_PATTERN_ELEMENTS + 1u];
	uint32_t sum_narrow = 0u;
	uint32_t split;
	uint8_t wide_count = 0u;
	uint8_t i;
	uint8_t j;

	for (i = 0u; i < BARCODE_PATTERN_ELEMENTS; ++i) {
		uint32_t width = widths[i];
		j = i;
		while ((j > 0u) && (sorted[j - 1u] > width)) {
			sorted[j] = sorted[j - 1u];
			--j;
		}
		sorted[j] = width;
	}

	if ((sorted[5] == 0u) ||
	    ((uint64_t)sorted[6] * 100u <
	     (uint64_t)sorted[5] * BARCODE_WIDE_NARROW_MIN_RATIO_X100) ||
	    ((uint64_t)sorted[8] > (uint64_t)sorted[0] * 5u)) {
		return false;
	}

	split = sorted[5] + (sorted[6] - sorted[5]) / 2u;
	for (i = 0u; i < BARCODE_PATTERN_ELEMENTS; ++i) {
		uint8_t position = reversed
			? (uint8_t)(BARCODE_PATTERN_ELEMENTS - 1u - i) : i;
		if (widths[i] > split) {
			pattern[position] = 'W';
			++wide_count;
		} else {
			pattern[position] = 'N';
			sum_narrow += widths[i];
		}
	}
	pattern[BARCODE_PATTERN_ELEMENTS] = '\0';
	if (wide_count != 3u) {
		return false;
	}

	*narrow_width = sum_narrow / 6u;
	for (i = 0u; i < 27u; ++i) {
		if (strcmp(pattern, code39_patterns[i]) == 0) {
			*symbol = code39_characters[i];
			return true;
		}
	}
	return false;
}

static bool decode_orientation(const uint32_t *widths, uint16_t width_count,
			       bool reversed,
			       barcode_decode_result_t *result)
{
	uint16_t character_count;
	uint16_t stop_position;
	uint16_t i;
	uint32_t narrow;
	char symbol;

	if ((width_count < 29u) || (((width_count - 19u) % 10u) != 0u)) {
		result->error = BARCODE_DECODE_BAD_LENGTH;
		return false;
	}
	character_count = (width_count - 19u) / 10u;
	if (character_count == 0u) {
		result->error = BARCODE_DECODE_BAD_LENGTH;
		return false;
	}
	if (character_count > BARCODE_MAX_CHARACTERS) {
		result->error = BARCODE_DECODE_TOO_LONG;
		return false;
	}

	if (!decode_symbol(widths, reversed, &symbol, &narrow) ||
	    (symbol != '*')) {
		result->error = BARCODE_DECODE_BAD_FRAME;
		result->error_character = 0u;
		return false;
	}

	for (i = 0u; i <= character_count; ++i) {
		uint16_t gap_position = (uint16_t)(9u + (10u * i));
		uint32_t gap = widths[gap_position];
		uint64_t min_gap = ((uint64_t)narrow * 2u) / 5u;
		uint64_t max_gap = (uint64_t)narrow *
			BARCODE_GAP_MAX_NARROW_RATIO_X100 / 100u;

		if (((uint64_t)gap < min_gap) || ((uint64_t)gap > max_gap)) {
			result->error = BARCODE_DECODE_BAD_WIDTH;
			result->error_character = i;
			return false;
		}
		if (i < character_count) {
			uint16_t symbol_position = (uint16_t)(10u + (10u * i));

			if (!decode_symbol(&widths[symbol_position], reversed,
					   &symbol, &narrow)) {
				result->error = BARCODE_DECODE_BAD_WIDTH;
				result->error_character = (uint16_t)(i + 1u);
				return false;
			}
			if ((symbol < 'A') || (symbol > 'Z')) {
				result->error = BARCODE_DECODE_BAD_PATTERN;
				result->error_character = (uint16_t)(i + 1u);
				return false;
			}
			result->text[i] = symbol;
		}
	}

	stop_position = (uint16_t)(10u + (10u * character_count));
	if (!decode_symbol(&widths[stop_position], reversed, &symbol, &narrow) ||
	    (symbol != '*')) {
		result->error = BARCODE_DECODE_BAD_FRAME;
		result->error_character = (uint16_t)(character_count + 1u);
		return false;
	}

	if (reversed) {
		for (i = 0u; i < character_count / 2u; ++i) {
			char temporary = result->text[i];
			result->text[i] = result->text[character_count - 1u - i];
			result->text[character_count - 1u - i] = temporary;
		}
	}
	result->text[character_count] = '\0';
	result->valid = true;
	result->reversed = reversed;
	result->elements = (uint16_t)(character_count * BARCODE_PATTERN_ELEMENTS);
	result->error = BARCODE_DECODE_OK;
	return true;
}

void barcode_decode_widths(const uint32_t *widths, uint16_t width_count,
			   barcode_decode_result_t *result)
{
	uint16_t i;

	if (result == NULL) {
		return;
	}
	(void)memset(result, 0, sizeof(*result));
	if ((widths == NULL) || (width_count == 0u)) {
		result->error = BARCODE_DECODE_BAD_LENGTH;
		return;
	}

	if (decode_orientation(widths, width_count, false, result)) {
		return;
	}
	{
		barcode_decode_result_t reverse_result;

		(void)memset(&reverse_result, 0, sizeof(reverse_result));
		if (decode_orientation(widths, width_count, true, &reverse_result)) {
			*result = reverse_result;
			result->reversed = true;
			return;
		}
		if ((reverse_result.error == BARCODE_DECODE_BAD_FRAME) &&
		    (result->error != BARCODE_DECODE_BAD_WIDTH)) {
			result->error = BARCODE_DECODE_BAD_FRAME;
		}
	}
	for (i = 0u; i < sizeof(result->text); ++i) {
		result->text[i] = '\0';
	}
}

const char *barcode_decode_error_name(barcode_decode_error_t error)
{
	switch (error) {
	case BARCODE_DECODE_BAD_LENGTH:
		return "bad_length";
	case BARCODE_DECODE_BAD_WIDTH:
		return "bad_width";
	case BARCODE_DECODE_BAD_PATTERN:
		return "bad_pattern";
	case BARCODE_DECODE_BAD_FRAME:
		return "bad_frame";
	case BARCODE_DECODE_TOO_LONG:
		return "too_long";
	case BARCODE_DECODE_OK:
	default:
		return "unknown";
	}
}
