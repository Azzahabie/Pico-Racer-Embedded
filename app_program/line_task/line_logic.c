/*
 * line_logic.c - two-sensor digital line reader (RP2040)
 *
 * Works in the micro T-Kernel build: it does NOT use the Pico SDK
 * (pico/stdlib.h, hardware/adc.h are not on this build's include path).
 * The pins are configured and read directly through the RP2040 registers.
 *
 * The sensors are digital (DO) outputs on GP6 and GP8, so there is no
 * ADC, threshold or hysteresis here: the sensor module itself decides
 * black or white, and this file reads the resulting 0/1 level.
 */

#include <stddef.h>

#include "line_logic.h"

/* ------------------------ RP2040 register access ----------------------- */

#define IO_BANK0_BASE   0x40014000UL
#define PADS_BANK0_BASE 0x4001C000UL
#define SIO_BASE        0xD0000000UL

#define REG32(addr)     (*(volatile uint32_t *)(addr))

#define GPIO_CTRL(pin)  REG32(IO_BANK0_BASE + 0x04UL + 8UL * (pin))
#define PAD_CTRL(pin)   REG32(PADS_BANK0_BASE + 0x04UL + 4UL * (pin))
#define SIO_GPIO_IN     REG32(SIO_BASE + 0x04UL)
#define SIO_GPIO_OE_CLR REG32(SIO_BASE + 0x28UL)

#define FUNCSEL_SIO     5u
#define PAD_IE          (1u << 6)   /* input enable */
#define PAD_PUE         (1u << 3)   /* pull-up      */
#define PAD_PDE         (1u << 2)   /* pull-down    */

typedef struct {
    line_reader_config_t config;
    line_reader_raw_t last_raw;
    uint8_t black_bits;
} line_reader_state_t;

static line_reader_state_t state;

/** Sets a pin to plain GPIO input with input enabled and both pulls off. */
static void input_pin_init(uint32_t pin)
{
    uint32_t pad;

    GPIO_CTRL(pin) = FUNCSEL_SIO;
    SIO_GPIO_OE_CLR = (1UL << pin);

    /* The pad powers up with a pull-down on, which would drag a weakly
       driven sensor output low. */
    pad = PAD_CTRL(pin);
    pad |= PAD_IE;
    pad &= ~(PAD_PUE | PAD_PDE);
    PAD_CTRL(pin) = pad;
}

static uint16_t pin_level(uint32_t pin)
{
    return (uint16_t)((SIO_GPIO_IN >> pin) & 1UL);
}

/* ------------------------------- public ------------------------------- */

line_reader_config_t line_reader_default_config(void)
{
    line_reader_config_t config = {
        .right_guard_black_is_high = (LINE_BLACK_IS_HIGH != 0u),
        .left_guard_black_is_high = (LINE_BLACK_IS_HIGH != 0u),
    };

    return config;
}

void line_reader_init(const line_reader_config_t *config)
{
    state.config = (config != NULL) ? *config : line_reader_default_config();
    state.black_bits = 0u;
    state.last_raw.right_guard = 0u;
    state.last_raw.left_guard = 0u;

    input_pin_init(LINE_RIGHT_GPIO);
    input_pin_init(LINE_LEFT_GPIO);
}

line_reader_raw_t line_reader_read_raw(void)
{
    line_reader_raw_t raw;

    raw.left_guard = pin_level(LINE_LEFT_GPIO);
    raw.right_guard = pin_level(LINE_RIGHT_GPIO);

    return raw;
}

uint8_t line_reader_read_2bit(void)
{
    line_reader_raw_t raw = line_reader_read_raw();
    bool right_guard_black;
    bool left_guard_black;

    state.last_raw = raw;

    right_guard_black =
        (raw.right_guard != 0u) == state.config.right_guard_black_is_high;
    left_guard_black =
        (raw.left_guard != 0u) == state.config.left_guard_black_is_high;

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