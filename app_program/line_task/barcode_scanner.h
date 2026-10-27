#ifndef ROBOCAR_BARCODE_SCANNER_H
#define ROBOCAR_BARCODE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../common/robot_config.h"
#include "barcode_decode.h"

typedef struct {
	bool black_is_high;
} barcode_scanner_config_t;

/**
 * Creates a barcode-sensor configuration using the board polarity.
 */
barcode_scanner_config_t barcode_scanner_default_config(void);

/**
 * Configures the barcode sensor GPIO as a digital input and creates the
 * decoded-result mutex.
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

/**
 * Samples the sensor and returns true once a complete scan or scan error is
 * available. Widths are timestamped from the RP2040 hardware timer in us.
 */
bool barcode_scanner_poll(barcode_decode_result_t *result);

bool barcode_scanner_scan_active(void);
ER barcode_scanner_last_error(void);

ER barcode_scanner_publish_decoded(const char *text);
ER barcode_scanner_get_decoded(char *output, uint16_t output_size,
			       uint16_t *length);

/**
 * Compatibility accessor; truncates to fit output and returns copied length.
 * Returns UINT16_MAX if the decoded-result mutex operation fails.
 */
uint16_t barcode_scanner_copy_decoded(char *output, uint16_t output_size);

#endif