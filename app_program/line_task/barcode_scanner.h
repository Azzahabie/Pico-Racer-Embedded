#ifndef ROBOCAR_BARCODE_SCANNER_H
#define ROBOCAR_BARCODE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

/* The barcode sensor is an analogue input on GP26, which is ADC channel 0.
 * Only GP26-GP29 have an ADC: GP26=ch0, GP27=ch1, GP28=ch2, GP29=ch3.
 * If the pin changes, change BOTH numbers together. */
#define BARCODE_SCANNER_GPIO        (26u)
#define BARCODE_SCANNER_ADC_CHANNEL (0u)

typedef struct {
    uint16_t threshold;
    uint16_t hysteresis;

    /* Set true when a reading above the threshold means black. */
    bool black_is_high;
} barcode_scanner_config_t;

/**
 * Creates a barcode-sensor configuration with safe 12-bit ADC starting values.
 * The threshold should be calibrated on the actual black and white surfaces.
 *
 * @return A complete configuration for barcode_scanner_init().
 */
barcode_scanner_config_t barcode_scanner_default_config(void);

/**
 * Initializes the ADC and configures GP26 as an analogue input.
 *
 * @param config Threshold, hysteresis and signal polarity. Pass NULL to use
 *               barcode_scanner_default_config(). The configuration is copied.
 */
void barcode_scanner_init(const barcode_scanner_config_t *config);

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

#endif 