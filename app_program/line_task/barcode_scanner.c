/*
 * barcode_scanner.c - GP26 ADC edge capture and Code 39 A-Z decoder.
 * The RP2040 ADC and timer are accessed directly for the micro T-Kernel build.
 */

#include <stddef.h>
#include <string.h>

#include "barcode_scanner.h"

#define BARCODE_PATTERN_ELEMENTS    (9u)
#define TIMER_BASE                  0x40054000UL
#define TIMER_RAW_HIGH              REG32(TIMER_BASE + 0x24UL)
#define TIMER_RAW_LOW               REG32(TIMER_BASE + 0x28UL)

#define REG32(addr)                 (*(volatile uint32_t *)(addr))

/* Atomic alias offsets: +0x2000 sets bits, +0x3000 clears bits. */
#define ALIAS_CLR                   0x3000UL
#define RESETS_BASE                 0x4000C000UL
#define RESETS_RESET_CLR            REG32(RESETS_BASE + ALIAS_CLR + 0x00UL)
#define RESETS_RESET_DONE           REG32(RESETS_BASE + 0x08UL)
#define RESET_ADC_BIT               (1u << 0)
#define CLOCKS_BASE                 0x40008000UL
#define CLK_ADC_CTRL                REG32(CLOCKS_BASE + 0x60UL)
#define CLK_ADC_ENABLE              (1u << 11)
#define IO_BANK0_BASE               0x40014000UL
#define PADS_BANK0_BASE             0x4001C000UL
#define GPIO_CTRL(pin)              REG32(IO_BANK0_BASE + 0x04UL + 8UL * (pin))
#define PAD_CTRL_CLR(pin)           REG32(PADS_BANK0_BASE + ALIAS_CLR + 0x04UL + 4UL * (pin))
#define FUNCSEL_NULL                0x1Fu
#define PAD_IE                      (1u << 6)
#define PAD_PUE                     (1u << 3)
#define PAD_PDE                     (1u << 2)
#define ADC_BASE                    0x4004C000UL
#define ADC_CS                      REG32(ADC_BASE + 0x00UL)
#define ADC_RESULT                  REG32(ADC_BASE + 0x04UL)
#define ADC_CS_EN                   (1u << 0)
#define ADC_CS_START_ONCE           (1u << 2)
#define ADC_CS_READY                (1u << 8)
#define ADC_CS_AINSEL_SHIFT         12u
#define HW_WAIT_LIMIT               200000UL

static const char code39_patterns[27][BARCODE_PATTERN_ELEMENTS + 1u] = {
    "WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",
    "WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",
    "NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",
    "WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",
    "NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",
    "WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",
    "WWNNWNNNN", "NWWNWNNNN", "NWNNWNWNN"
};

static const char code39_characters[28] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ*";

typedef struct {
    barcode_scanner_config_t config;
    uint16_t last_raw;
    bool black;
    bool scanning;
    bool last_edge_black;
    uint32_t minimum_width_us;
    uint64_t scan_started_us;
    uint64_t last_edge_us;
    uint32_t widths[BARCODE_MAX_WIDTHS];
    size_t width_count;
    char decoded[BARCODE_MAX_CHARACTERS + 1u];
} barcode_scanner_state_t;

static barcode_scanner_state_t state;

static bool wait_set(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t guard = HW_WAIT_LIMIT;

    while (((*reg) & mask) == 0u) {
        if (guard-- == 0u) {
            return false;
        }
    }
    return true;
}

static void adc_hw_init(void)
{
    RESETS_RESET_CLR = RESET_ADC_BIT;
    (void)wait_set(&RESETS_RESET_DONE, RESET_ADC_BIT);

    if ((CLK_ADC_CTRL & CLK_ADC_ENABLE) == 0u) {
        CLK_ADC_CTRL = CLK_ADC_ENABLE;
    }

    ADC_CS = ADC_CS_EN;
    (void)wait_set(&ADC_CS, ADC_CS_READY);
}

static void adc_pin_init(uint32_t pin)
{
    GPIO_CTRL(pin) = FUNCSEL_NULL;
    PAD_CTRL_CLR(pin) = PAD_IE | PAD_PUE | PAD_PDE;
}

static uint16_t adc_read_once(void)
{
    ADC_CS = ADC_CS_EN | ADC_CS_START_ONCE |
             ((uint32_t)BARCODE_ADC_CHANNEL << ADC_CS_AINSEL_SHIFT);
    (void)wait_set(&ADC_CS, ADC_CS_READY);
    return (uint16_t)(ADC_RESULT & 0x0FFFu);
}

static uint16_t read_average(void)
{
    uint32_t total = 0u;
    uint32_t sample;

    for (sample = 0u; sample < BARCODE_ADC_SAMPLES; ++sample) {
        total += adc_read_once();
    }
    return (uint16_t)(total / BARCODE_ADC_SAMPLES);
}

static uint64_t timer_now_us(void)
{
    uint32_t high_before;
    uint32_t high_after;
    uint32_t low;

    do {
        high_before = TIMER_RAW_HIGH;
        low = TIMER_RAW_LOW;
        high_after = TIMER_RAW_HIGH;
    } while (high_before != high_after);

    return ((uint64_t)high_before << 32) | low;
}

static bool detect_black(uint16_t reading)
{
    if (state.config.black_is_high) {
        return state.black
                   ? (reading > state.config.white_falling_threshold)
                   : (reading >= state.config.black_rising_threshold);
    }
    return state.black
               ? (reading < state.config.black_rising_threshold)
               : (reading <= state.config.white_falling_threshold);
}

static bool decode_symbol(const uint32_t *widths, bool reversed,
                          char *symbol, uint32_t *narrow_width)
{
    uint32_t sorted[BARCODE_PATTERN_ELEMENTS];
    char pattern[BARCODE_PATTERN_ELEMENTS + 1u];
    uint32_t sum_narrow = 0u;
    uint32_t split;
    uint8_t wide_count = 0u;
    uint8_t i;
    uint8_t j;

    for (i = 0u; i < BARCODE_PATTERN_ELEMENTS; ++i) {
        uint32_t width = widths[i];
        j = i;
        while ((j > 0u) && (sorted[j - 1u] > width)) {
            sorted[j] = sorted[j - 1u];
            --j;
        }
        sorted[j] = width;
    }

    if ((sorted[5] == 0u) ||
        ((uint64_t)sorted[6] * 3u < (uint64_t)sorted[5] * 5u) ||
        ((uint64_t)sorted[8] > (uint64_t)sorted[0] * 5u)) {
        return false;
    }

    split = sorted[5] + (sorted[6] - sorted[5]) / 2u;
    for (i = 0u; i < BARCODE_PATTERN_ELEMENTS; ++i) {
        uint8_t position = reversed
                               ? (uint8_t)(BARCODE_PATTERN_ELEMENTS - 1u - i)
                               : i;
        if (widths[i] > split) {
            pattern[position] = 'W';
            ++wide_count;
        } else {
            pattern[position] = 'N';
            sum_narrow += widths[i];
        }
    }
    pattern[BARCODE_PATTERN_ELEMENTS] = '\0';
    if (wide_count != 3u) {
        return false;
    }

    *narrow_width = sum_narrow / 6u;
    for (i = 0u; i < 27u; ++i) {
        if (strcmp(pattern, code39_patterns[i]) == 0) {
            *symbol = code39_characters[i];
            return true;
        }
    }
    return false;
}

static bool decode_orientation(const uint32_t *widths, uint16_t width_count,
                               bool reversed, barcode_decode_result_t *result)
{
    uint16_t character_count;
    uint16_t stop_position;
    uint16_t i;
    uint32_t narrow;
    char symbol;

    if ((width_count < 29u) || (((width_count - 19u) % 10u) != 0u)) {
        result->error = BARCODE_DECODE_BAD_LENGTH;
        return false;
    }
    character_count = (width_count - 19u) / 10u;
    if (character_count == 0u) {
        result->error = BARCODE_DECODE_BAD_LENGTH;
        return false;
    }
    if (character_count > BARCODE_MAX_CHARACTERS) {
        result->error = BARCODE_DECODE_TOO_LONG;
        return false;
    }

    if (!decode_symbol(widths, reversed, &symbol, &narrow) || (symbol != '*')) {
        result->error = BARCODE_DECODE_BAD_FRAME;
        result->error_character = 0u;
        return false;
    }

    for (i = 0u; i <= character_count; ++i) {
        uint16_t gap_position = (uint16_t)(9u + (10u * i));
        uint32_t gap = widths[gap_position];
        uint64_t min_gap = ((uint64_t)narrow * 2u) / 5u;
        uint64_t max_gap = (uint64_t)narrow * 5u / 2u;

        if (((uint64_t)gap < min_gap) || ((uint64_t)gap > max_gap)) {
            result->error = BARCODE_DECODE_BAD_WIDTH;
            result->error_character = (uint16_t)i;
            return false;
        }
        if (i < character_count) {
            uint16_t symbol_position = (uint16_t)(10u + (10u * i));

            if (!decode_symbol(&widths[symbol_position], reversed,
                               &symbol, &narrow)) {
                result->error = BARCODE_DECODE_BAD_WIDTH;
                result->error_character = (uint16_t)(i + 1u);
                return false;
            }
            if ((symbol < 'A') || (symbol > 'Z')) {
                result->error = BARCODE_DECODE_BAD_PATTERN;
                result->error_character = (uint16_t)(i + 1u);
                return false;
            }
            result->text[i] = symbol;
        }
    }

    stop_position = (uint16_t)(10u + (10u * character_count));
    if (!decode_symbol(&widths[stop_position], reversed, &symbol, &narrow) ||
        (symbol != '*')) {
        result->error = BARCODE_DECODE_BAD_FRAME;
        result->error_character = (uint16_t)(character_count + 1u);
        return false;
    }

    if (reversed) {
        for (i = 0u; i < character_count / 2u; ++i) {
            char temporary = result->text[i];
            result->text[i] = result->text[character_count - 1u - i];
            result->text[character_count - 1u - i] = temporary;
        }
    }
    result->text[character_count] = '\0';
    result->valid = true;
    result->reversed = reversed;
    result->elements = (uint16_t)(character_count * BARCODE_PATTERN_ELEMENTS);
    result->error = BARCODE_DECODE_OK;
    return true;
}

static void store_decoded(const barcode_decode_result_t *result)
{
    if (result->valid) {
        (void)strncpy(state.decoded, result->text, sizeof(state.decoded) - 1u);
        state.decoded[sizeof(state.decoded) - 1u] = '\0';
    }
}

static void finish_scan(barcode_decode_result_t *result)
{
    barcode_decode_widths(state.widths, (uint16_t)state.width_count, result);
    store_decoded(result);
}

barcode_scanner_config_t barcode_scanner_default_config(void)
{
    barcode_scanner_config_t config = {
        .black_rising_threshold = BARCODE_BLACK_RISING_THRESHOLD,
        .white_falling_threshold = BARCODE_WHITE_FALLING_THRESHOLD,
        .black_is_high = (BARCODE_BLACK_IS_HIGH != 0u),
    };
    return config;
}

void barcode_scanner_init(const barcode_scanner_config_t *config)
{
    state.config = (config != NULL) ? *config : barcode_scanner_default_config();
    state.last_raw = 0u;
    state.black = false;
    state.scanning = false;
    state.last_edge_black = false;
    state.minimum_width_us = UINT32_MAX;
    state.scan_started_us = 0u;
    state.last_edge_us = 0u;
    state.width_count = 0u;
    state.decoded[0] = '\0';
    adc_hw_init();
    adc_pin_init(BARCODE_GPIO);
}

uint16_t barcode_scanner_read_raw(void)
{
    return read_average();
}

bool barcode_scanner_read_black(void)
{
    state.last_raw = barcode_scanner_read_raw();
    state.black = detect_black(state.last_raw);
    return state.black;
}

uint16_t barcode_scanner_get_last_raw(void)
{
    return state.last_raw;
}

bool barcode_scanner_poll(barcode_decode_result_t *result)
{
    uint64_t now;
    bool black;

    if (result == NULL) {
        return false;
    }

    black = barcode_scanner_read_black();
    now = timer_now_us();
    if (!state.scanning) {
        if (black) {
            state.scanning = true;
            state.last_edge_black = true;
            state.minimum_width_us = UINT32_MAX;
            state.scan_started_us = now;
            state.last_edge_us = now;
            state.width_count = 0u;
        }
        return false;
    }

    if ((now - state.scan_started_us) >= BARCODE_SCAN_MAX_US) {
        result->valid = false;
        result->error = BARCODE_DECODE_TOO_LONG;
        result->error_character = 0u;
        state.scanning = false;
        return true;
    }

    if (black != state.last_edge_black) {
        uint64_t elapsed = now - state.last_edge_us;

        if (state.width_count >= BARCODE_MAX_WIDTHS) {
            result->valid = false;
            result->error = BARCODE_DECODE_TOO_LONG;
            result->error_character = 0u;
            state.scanning = false;
            return true;
        }
        state.widths[state.width_count++] =
            (elapsed > UINT32_MAX) ? UINT32_MAX : (uint32_t)elapsed;
        if (state.widths[state.width_count - 1u] < state.minimum_width_us) {
            state.minimum_width_us = state.widths[state.width_count - 1u];
        }
        state.last_edge_us = now;
        state.last_edge_black = black;
        if (!black) {
            barcode_decode_result_t candidate;

            barcode_decode_widths(state.widths,
                                  (uint16_t)state.width_count, &candidate);
            if (candidate.valid) {
                store_decoded(&candidate);
                *result = candidate;
                state.scanning = false;
                return true;
            }
        }
        return false;
    }

    {
        uint64_t idle_timeout = BARCODE_SCAN_IDLE_US;
        uint64_t width_timeout =
            (state.minimum_width_us == UINT32_MAX)
                ? 0u
                : (uint64_t)state.minimum_width_us * 4u;

        if (width_timeout > idle_timeout) {
            idle_timeout = width_timeout;
        }
        if (!black && ((now - state.last_edge_us) >= idle_timeout)) {
            finish_scan(result);
            state.scanning = false;
            return true;
        }
    }
    return false;
}

void barcode_decode_widths(const uint32_t *widths, uint16_t width_count,
                           barcode_decode_result_t *result)
{
    uint16_t i;

    if (result == NULL) {
        return;
    }
    (void)memset(result, 0, sizeof(*result));
    if ((widths == NULL) || (width_count == 0u)) {
        result->error = BARCODE_DECODE_BAD_LENGTH;
        return;
    }

    if (decode_orientation(widths, width_count, false, result)) {
        return;
    }
    {
        barcode_decode_result_t reverse_result;

        (void)memset(&reverse_result, 0, sizeof(reverse_result));
        if (decode_orientation(widths, width_count, true, &reverse_result)) {
            *result = reverse_result;
            result->reversed = true;
            return;
        }
        if ((reverse_result.error == BARCODE_DECODE_BAD_FRAME) &&
            (result->error != BARCODE_DECODE_BAD_WIDTH)) {
            result->error = BARCODE_DECODE_BAD_FRAME;
        }
    }
    for (i = 0u; i < sizeof(result->text); ++i) {
        result->text[i] = '\0';
    }
}

uint16_t barcode_scanner_copy_decoded(char *output, uint16_t output_size)
{
    uint16_t length = (uint16_t)strlen(state.decoded);

    if ((output == NULL) || (output_size == 0u)) {
        return length;
    }
    if (length >= output_size) {
        length = output_size - 1u;
    }
    (void)memcpy(output, state.decoded, length);
    output[length] = '\0';
    return length;
}

const char *barcode_decode_error_name(barcode_decode_error_t error)
{
    switch (error) {
    case BARCODE_DECODE_BAD_LENGTH:
        return "bad_length";
    case BARCODE_DECODE_BAD_WIDTH:
        return "bad_width";
    case BARCODE_DECODE_BAD_PATTERN:
        return "bad_pattern";
    case BARCODE_DECODE_BAD_FRAME:
        return "bad_frame";
    case BARCODE_DECODE_TOO_LONG:
        return "too_long";
    case BARCODE_DECODE_OK:
    default:
        return "unknown";
    }
}
