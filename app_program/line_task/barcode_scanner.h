#ifndef ROBOCAR_BARCODE_SCANNER_H
#define ROBOCAR_BARCODE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "line_config.h"

#define BARCODE_SCANNER_GPIO        BARCODE_SENSOR_GPIO
#define BARCODE_SCANNER_ADC_CHANNEL BARCODE_SENSOR_ADC_CHANNEL
#define BARCODE_SCANNER_RESULT_SIZE (BARCODE_MAX_CHARACTERS + 1u)

typedef struct {
    uint16_t black_rising_threshold;
    uint16_t black_falling_threshold;
    bool black_is_high;
} barcode_scanner_config_t;

/**
 * Creates the configured 12-bit ADC thresholds and signal polarity.
 *
 * @return A complete configuration for barcode_scanner_init().
 */
barcode_scanner_config_t barcode_scanner_default_config(void);

/**
 * Initializes the ADC and configures GP26 as an analogue input.
 *
 * @param config Edge thresholds and signal polarity. Pass NULL to use
 *               barcode_scanner_default_config(). The configuration is copied.
 */
ER barcode_scanner_init(const barcode_scanner_config_t *config);

/**
 * Reads and averages the barcode sensor.
 *
 * @return The raw 12-bit ADC reading, in the range 0-4095.
 */
uint16_t barcode_scanner_read_raw(void);

/**
 * Reads GP26 and classifies the surface using hysteresis. This sensor is only
 * used to capture barcode transitions and does not control line steering.
 *
 * @return true when the barcode sensor sees black; otherwise false.
 */
bool barcode_scanner_read_black(void);

/** Returns the raw ADC value captured by the most recent classified read. */
uint16_t barcode_scanner_get_last_raw(void);

/** Publishes the latest decoded text for other tasks to read safely. */
ER barcode_scanner_publish_decoded(const char *text);

/** Copies the most recently decoded text, including its terminator. */
ER barcode_scanner_get_decoded(char *buffer, uint32_t buffer_size);

#endif