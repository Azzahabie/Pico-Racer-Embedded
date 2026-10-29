#ifndef ROBOCAR_BARCODE_DECODE_H
#define ROBOCAR_BARCODE_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"

typedef enum {
	BARCODE_DECODE_OK = 0,
	BARCODE_DECODE_BAD_LENGTH,
	BARCODE_DECODE_BAD_WIDTH,
	BARCODE_DECODE_BAD_PATTERN,
	BARCODE_DECODE_BAD_FRAME,
	BARCODE_DECODE_TOO_LONG
} barcode_decode_error_t;

typedef struct {
	bool valid;
	bool reversed;
	barcode_decode_error_t error;
	uint16_t error_character;
	uint16_t elements;
	char text[BARCODE_MAX_CHARACTERS + 1u];
} barcode_decode_result_t;

void barcode_decode_widths(const uint32_t *widths, uint16_t width_count,
			   barcode_decode_result_t *result);
const char *barcode_decode_error_name(barcode_decode_error_t error);

#endif
