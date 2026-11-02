#include "adc_hw.h"
#include "line_logic.h"

typedef struct {
    line_reader_config_t config;
    line_reader_raw_t last_raw;
    uint8_t black_bits;
    ER last_error;
} line_reader_state_t;

static line_reader_state_t state;

line_reader_config_t line_reader_default_config(void)
{
    line_reader_config_t config = {
        .right_guard_black_is_high =
            (LINE_SENSOR_RIGHT_BLACK_IS_HIGH != 0u),
        .left_guard_black_is_high =
            (LINE_SENSOR_LEFT_BLACK_IS_HIGH != 0u),
        .right_guard_black_rising_threshold =
            LINE_SENSOR_RIGHT_BLACK_RISING_THRESHOLD,
        .right_guard_white_falling_threshold =
            LINE_SENSOR_RIGHT_BLACK_FALLING_THRESHOLD,
        .left_guard_black_rising_threshold =
            LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
        .left_guard_white_falling_threshold =
            LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD,
    };

    return config;
}

ER line_reader_init(const line_reader_config_t *config)
{
    ER result = adc_hw_init();

    if (result < E_OK) {
        return result;
    }

    state.config = (config != NULL) ? *config : line_reader_default_config();
    state.black_bits = 0u;
    state.last_raw.right_guard = 0u;
    state.last_raw.left_guard = 0u;
    state.last_error = E_OK;

    adc_hw_pin_init(LINE_SENSOR_RIGHT_GPIO);
    adc_hw_pin_init(LINE_SENSOR_LEFT_GPIO);
    return E_OK;
}

line_reader_raw_t line_reader_read_raw(void)
{
    line_reader_raw_t raw;

    raw.left_guard = adc_hw_read(LINE_SENSOR_LEFT_ADC_CHANNEL);
    state.last_error = adc_hw_last_error();
    raw.right_guard = adc_hw_read(LINE_SENSOR_RIGHT_ADC_CHANNEL);
    if (state.last_error >= E_OK) {
        state.last_error = adc_hw_last_error();
    }

    return raw;
}

ER line_reader_last_error(void)
{
    return state.last_error;
}

uint8_t line_reader_read_2bit(void)
{
    line_reader_raw_t raw = line_reader_read_raw();
    bool right_guard_black;
    bool left_guard_black;

    state.last_raw = raw;
    if ((raw.left_guard > 0x0FFFu) || (raw.right_guard > 0x0FFFu)) {
        return state.black_bits;
    }

    if (state.config.right_guard_black_is_high) {
        right_guard_black = state.black_bits & LINE_READER_RIGHT_GUARD_MASK
            ? raw.right_guard > state.config.right_guard_white_falling_threshold
            : raw.right_guard >= state.config.right_guard_black_rising_threshold;
    } else {
        right_guard_black = state.black_bits & LINE_READER_RIGHT_GUARD_MASK
            ? raw.right_guard < state.config.right_guard_black_rising_threshold
            : raw.right_guard <= state.config.right_guard_white_falling_threshold;
    }
    if (state.config.left_guard_black_is_high) {
        left_guard_black = state.black_bits & LINE_READER_LEFT_GUARD_MASK
            ? raw.left_guard > state.config.left_guard_white_falling_threshold
            : raw.left_guard >= state.config.left_guard_black_rising_threshold;
    } else {
        left_guard_black = state.black_bits & LINE_READER_LEFT_GUARD_MASK
            ? raw.left_guard < state.config.left_guard_black_rising_threshold
            : raw.left_guard <= state.config.left_guard_white_falling_threshold;
    }

    state.black_bits =
        (uint8_t)((right_guard_black ? LINE_READER_RIGHT_GUARD_MASK : 0u) |
                  (left_guard_black ? LINE_READER_LEFT_GUARD_MASK : 0u));

    return state.black_bits;
}

line_reader_raw_t line_reader_get_last_raw(void)
{
    return state.last_raw;
}

void line_reader_format_2bit(uint8_t value, char output[3])
{
    value &= LINE_READER_OUTPUT_MASK;
    output[0] = ((value & LINE_READER_RIGHT_GUARD_MASK) != 0u) ? '1' : '0';
    output[1] = ((value & LINE_READER_LEFT_GUARD_MASK) != 0u) ? '1' : '0';
    output[2] = '\0';
}