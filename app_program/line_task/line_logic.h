#ifndef ROBOCAR_LINE_LOGIC_H
#define ROBOCAR_LINE_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../common/robot_config.h"

/* Two-bit output layout: right guard in bit 1 and left guard in bit 0. */
#define LINE_READER_RIGHT_GUARD_MASK (1u << 1)
#define LINE_READER_LEFT_GUARD_MASK  (1u << 0)
#define LINE_READER_OUTPUT_MASK      (0x03u)

typedef struct {
    uint16_t right_guard;
    uint16_t left_guard;
} line_reader_raw_t;

typedef struct {
    bool right_guard_black_is_high;
    bool left_guard_black_is_high;
    uint16_t right_guard_black_rising_threshold;
    uint16_t right_guard_white_falling_threshold;
    uint16_t left_guard_black_rising_threshold;
    uint16_t left_guard_white_falling_threshold;
} line_reader_config_t;

/**
 * Creates a line-sensor configuration from the board thresholds and polarity.
 */
line_reader_config_t line_reader_default_config(void);

/**
 * Initializes the ADC and both analogue sensor pins.
 *
 * @param config Polarity per sensor. Pass NULL to use
 *               line_reader_default_config(). The configuration is copied.
 */
ER line_reader_init(const line_reader_config_t *config);

/** Reads 12-bit ADC values for the right and left sensors. */
line_reader_raw_t line_reader_read_raw(void);
ER line_reader_last_error(void);

/**
 * Classifies both line sensors.
 *
 * @return A packed 0bRL value. A set bit means that sensor sees black.
 */
uint8_t line_reader_read_2bit(void);

/** Returns the pin levels captured by the most recent classified read. */
line_reader_raw_t line_reader_get_last_raw(void);

/**
 * Converts a packed 0bRL value to a null-terminated string such as "10".
 *
 * @param value  Packed line-sensor value; bits above the lowest two are ignored.
 * @param output Caller-provided buffer of at least three characters.
 */
void line_reader_format_2bit(uint8_t value, char output[3]);

#endif