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

/* 1: GP7 wiring check.  ir_sense_task prints "[gp7]" every 500 ms with GP7
 * read under the pad's pull-down, pull-up and no pull.  A connected,
 * working DO reads the same in all three and follows the card; a floating
 * (unconnected) pin follows the pull; a stuck DO reads 1 in all three. */
#ifndef LINE_GP7_DIAG
#define LINE_GP7_DIAG	0
#endif

#if LINE_GP7_DIAG
/** Reads GP7 under each pull setting, then restores no pull. */
void barcode_scanner_probe(uint16_t *p_pd, uint16_t *p_pu, uint16_t *p_none);
#endif

#endif
