#ifndef ROBOCAR_BARCODE_SCANNER_H
#define ROBOCAR_BARCODE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_config.h"

typedef struct {
    uint16_t black_rising_threshold;
    uint16_t white_falling_threshold;
    bool black_is_high;
} barcode_scanner_config_t;

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

/**
 * Creates a barcode-sensor configuration using the board thresholds in
 * robot_config.h.
 *
 * @return A complete configuration for barcode_scanner_init().
 */
barcode_scanner_config_t barcode_scanner_default_config(void);

/**
 * Initializes the ADC and configures GP26 as an analogue input.
 *
 * @param config Rising/falling thresholds and signal polarity. Pass NULL to
 *               use barcode_scanner_default_config(). The configuration is copied.
 */
void barcode_scanner_init(const barcode_scanner_config_t *config);

/**
 * Reads the barcode sensor once.
 *
 * @return The raw 12-bit ADC reading, in the range 0-4095.
 */
uint16_t barcode_scanner_read_raw(void);

/**
 * Reads GP26 and classifies the surface using separate black/white thresholds.
 * This sensor only captures barcode transitions and does not steer.
 *
 * @return true when the barcode sensor sees black; otherwise false.
 */
bool barcode_scanner_read_black(void);

/** Returns the raw ADC value captured by the most recent classified read. */
uint16_t barcode_scanner_get_last_raw(void);

/**
 * Samples the sensor and returns true once a complete scan or scan error is
 * available. Widths are timestamped from the RP2040 hardware timer in us.
 */
bool barcode_scanner_poll(barcode_decode_result_t *result);

/**
 * Decodes a Code 39 A-Z scan from alternating bar/space widths in microseconds.
 * The array includes the start/stop '*' symbols and their inter-character gaps.
 */
void barcode_decode_widths(const uint32_t *widths, uint16_t width_count,
                           barcode_decode_result_t *result);

/** Copies the most recently decoded text into caller-provided storage. */
uint16_t barcode_scanner_copy_decoded(char *output, uint16_t output_size);

const char *barcode_decode_error_name(barcode_decode_error_t error);

#endif