#ifndef ROBOCAR_LINE_LOGIC_H
#define ROBOCAR_LINE_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include "line_config.h"

/* Two-bit output layout: right guard in bit 1 and left guard in bit 0. */
#define LINE_READER_RIGHT_GUARD_MASK (1u << 1)
#define LINE_READER_LEFT_GUARD_MASK  (1u << 0)
#define LINE_READER_OUTPUT_MASK      (0x03u)

/* Pin assignments are centralized in line_config.h. GP6 and GP8 are
 * digital (DO) inputs and cannot be used as ADC inputs. */
#define LINE_READER_RIGHT_GUARD_GPIO LINE_SENSOR_RIGHT_GPIO
#define LINE_READER_LEFT_GUARD_GPIO  LINE_SENSOR_LEFT_GPIO

/* Digital level of each sensor pin: 0 or 1 (NOT an ADC value). */
typedef struct {
    uint16_t right_guard;
    uint16_t left_guard;
} line_reader_raw_t;

typedef struct {
    /* Set a field true when a HIGH pin level means black. */
    bool right_guard_black_is_high;
    bool left_guard_black_is_high;
} line_reader_config_t;

/**
 * Creates a line-sensor configuration with HIGH = black on both sensors.
 * Flip a field to false if the module outputs LOW over black.
 */
line_reader_config_t line_reader_default_config(void);

/**
 * Sets up both sensor pins as inputs with pulls off.
 *
 * @param config Polarity per sensor. Pass NULL to use
 *               line_reader_default_config(). The configuration is copied.
 */
void line_reader_init(const line_reader_config_t *config);

/** Reads the current pin levels (0 or 1) of the right and left sensors. */
line_reader_raw_t line_reader_read_raw(void);

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