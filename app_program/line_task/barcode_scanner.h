#ifndef ROBOCAR_BARCODE_SCANNER_H
#define ROBOCAR_BARCODE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../common/robot_config.h"

typedef struct {
	bool black_is_high;
} barcode_scanner_config_t;

/**
 * Creates a barcode-sensor configuration using the board polarity.
 */
barcode_scanner_config_t barcode_scanner_default_config(void);

/**
 * Configures the barcode sensor GPIO as a digital input.
 *
 * @param config Signal polarity. Pass NULL to use the board configuration.
 */
ER barcode_scanner_init(const barcode_scanner_config_t *config);

/** Reads the digital input level; the raw result is always 0 or 1. */
uint16_t barcode_scanner_read_raw(void);

/**
 * Reads the GP7 DO input and applies the configured output polarity.
 *
 * @return true when the barcode sensor sees black; otherwise false.
 */
bool barcode_scanner_read_black(void);

/** Returns the digital level captured by the most recent read. */
uint16_t barcode_scanner_get_last_raw(void);

#endif
