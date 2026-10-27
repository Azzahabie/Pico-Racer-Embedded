/*
 * line_task.c - two digital line sensors and independent Code 39 reader.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include <stdint.h>

#include "../common/console_logger.h"
#include "../usb_console_compat.h"
#include "line_task.h"
#include "line_logic.h"
#include "barcode_scanner.h"
#include "barcode_decode.h"

#define BARCODE_TIMER_BASE (0x40054000UL)
#define BARCODE_TIMER_RAWL (*(volatile uint32_t *)(BARCODE_TIMER_BASE + 0x28UL))

typedef enum {
    LINE_LOST = 0,
    LINE_LEFT_DEVIATION,
    LINE_RIGHT_DEVIATION,
    LINE_JUNCTION
} line_state_t;

static line_state_t filtered_state = LINE_LOST;
static line_state_t candidate_state = LINE_LOST;
static UINT candidate_count;

static line_state_t bits_to_state(uint8_t bits)
{
    switch (bits & LINE_READER_OUTPUT_MASK) {
    case 0u:
        return LINE_LOST;
    case LINE_READER_LEFT_GUARD_MASK:
        return LINE_LEFT_DEVIATION;
    case LINE_READER_RIGHT_GUARD_MASK:
        return LINE_RIGHT_DEVIATION;
    default:
        return LINE_JUNCTION;
    }
}

static uint8_t line_state_bits(line_state_t state)
{
    switch (state) {
    case LINE_LEFT_DEVIATION:
        return LINE_READER_LEFT_GUARD_MASK;
    case LINE_RIGHT_DEVIATION:
        return LINE_READER_RIGHT_GUARD_MASK;
    case LINE_JUNCTION:
        return LINE_READER_OUTPUT_MASK;
    case LINE_LOST:
    default:
        return 0u;
    }
}

static const char *line_state_name(line_state_t state)
{
    switch (state) {
    case LINE_LOST:
        return "LOST (both white)";
    case LINE_LEFT_DEVIATION:
        return "LEFT DEVIATION";
    case LINE_RIGHT_DEVIATION:
        return "RIGHT DEVIATION";
    case LINE_JUNCTION:
        return "JUNCTION";
    default:
        return "INVALID";
    }
}

static const char *line_command(line_state_t state)
{
    switch (state) {
    case LINE_LOST:
#if LINE_STOP_ON_LOST
        return "STOP";
#else
        return "DRIVE";
#endif
    case LINE_LEFT_DEVIATION:
#if LINE_STEER_INVERT
        return "STEER_LEFT";
#else
        return "STEER_RIGHT";
#endif
    case LINE_RIGHT_DEVIATION:
#if LINE_STEER_INVERT
        return "STEER_RIGHT";
#else
        return "STEER_LEFT";
#endif
    case LINE_JUNCTION:
        return "DRIVE";
    default:
        return "INVALID";
    }
}

static line_state_t line_update(uint8_t *raw_bits)
{
    line_state_t raw_state;
    UINT required;

    raw_state = bits_to_state(line_reader_read_2bit());
    if (raw_state == candidate_state) {
        if (candidate_count < 1000u) {
            ++candidate_count;
        }
    } else {
        candidate_state = raw_state;
        candidate_count = 1u;
    }

    required = (candidate_state == LINE_LOST)
                   ? LINE_LOST_SAMPLES
                   : LINE_CONFIRM_SAMPLES;
    if (candidate_count >= required) {
        filtered_state = candidate_state;
    }
    *raw_bits = line_state_bits(filtered_state);

    return filtered_state;
}

#if LINE_COMMISSIONING_MODE
static void commissioning_phase(const char *surface)
{
    UINT samples;
    UINT i;

    samples = LINE_COMMISSION_PHASE_MS /
              LINE_COMMISSION_SAMPLE_PERIOD_MS;
    CONSOLE_LOG((UB *)"[line] commissioning: place sensors over %s for 5 s\n",
                surface);
    for (i = 0u; i < samples; ++i) {
        line_reader_raw_t raw = line_reader_read_raw();

        CONSOLE_LOG((UB *)"[line] raw GP%d=%d GP%d=%d (%s)\n",
                    (INT)LINE_READER_LEFT_GUARD_GPIO, (INT)raw.left_guard,
                    (INT)LINE_READER_RIGHT_GUARD_GPIO, (INT)raw.right_guard,
                    surface);
        tk_dly_tsk(LINE_COMMISSION_SAMPLE_PERIOD_MS);
    }
}
#endif

static void barcode_append_width(uint32_t widths[BARCODE_MAX_ELEMENTS],
                                 uint32_t *count,
                                 bool *overflow,
                                 uint32_t width)
{
    if (*count < BARCODE_MAX_ELEMENTS) {
        widths[*count] = width;
        ++(*count);
    } else {
        *overflow = true;
    }
}

static void barcode_report_scan(const uint32_t *widths,
                                uint32_t count,
                                bool overflow)
{
    barcode_decode_result_t result;
    ER error;

    if (overflow) {
        CONSOLE_LOG((UB *)"[barcode] ERROR: too_many_elements\n");
        return;
    }

    barcode_decode(widths, count, &result);
    if (!result.valid) {
        if (result.error_character != 0u) {
            CONSOLE_LOG((UB *)"[barcode] ERROR: %s at char %u\n",
                        barcode_error_name(result.error),
                        result.error_character);
        } else {
            CONSOLE_LOG((UB *)"[barcode] ERROR: %s\n",
                        barcode_error_name(result.error));
        }
        return;
    }

    error = barcode_scanner_publish_decoded(result.text);
    if (error < E_OK) {
        CONSOLE_LOG((UB *)"[barcode] ERROR: result_store er=%d\n", error);
        return;
    }

    CONSOLE_LOG((UB *)"[barcode] DECODED: \"%s\" (dir=%s, elems=%u)\n",
                result.text,
                (result.direction == BARCODE_DIRECTION_FORWARD) ? "fwd" : "rev",
                result.element_count);
}

void line_task(INT stacd, void *exinf)
{
    line_state_t state;
    line_state_t last_state = (line_state_t)4;
    uint32_t barcode_widths[BARCODE_MAX_ELEMENTS];
    uint32_t barcode_width_count = 0u;
    uint32_t segment_start_us = 0u;
    uint32_t last_edge_us = 0u;
    UINT heartbeat_ms = 0u;
    uint8_t raw_bits;
    char bit_text[3];
    bool barcode_ready;
    bool barcode_black;
    bool barcode_scan_active = false;
    bool barcode_overflow = false;
    bool barcode_current_black = false;
    INT i;

    (void)stacd;
    (void)exinf;

    for (i = 0; i < 200 && tm_usb_state() < 2u; ++i) {
        tk_dly_tsk(100);
    }
    tk_dly_tsk(800);

    CONSOLE_LOG((UB *)"\n=== Line reader started ===\n");
    CONSOLE_LOG((UB *)"Left sensor: GP%d, right sensor: GP%d (digital, 1=black)\n",
                (INT)LINE_READER_LEFT_GUARD_GPIO,
                (INT)LINE_READER_RIGHT_GUARD_GPIO);
    CONSOLE_LOG((UB *)"Barcode: GP%d ADC%d; battery sense reserved: GP%d ADC%d\n",
                (INT)BARCODE_SENSOR_GPIO, (INT)BARCODE_SENSOR_ADC_CHANNEL,
                (INT)BATTERY_SENSE_GPIO, (INT)BATTERY_SENSE_ADC_CHANNEL);

#if LINE_SENSOR_MOTOR_PIN_CONFLICT
    CONSOLE_LOG((UB *)"[pin conflict] line sensor GP%d overlaps motor driver "
                "GP%d-GP%d; resolve wiring before connecting hardware\n",
                (INT)LINE_SENSOR_LEFT_GPIO, (INT)MOTOR_DRIVER_GPIO_1,
                (INT)MOTOR_DRIVER_GPIO_4);
#endif
#if LINE_SENSOR_BATTERY_PIN_CONFLICT || BARCODE_BATTERY_PIN_CONFLICT
    CONSOLE_LOG((UB *)"[pin conflict] sensor and battery-sense assignments "
                "overlap; resolve wiring before connecting hardware\n");
#endif

    line_reader_init(NULL);
    {
        ER error = barcode_scanner_init(NULL);
        barcode_ready = (error >= E_OK);
        if (!barcode_ready) {
            CONSOLE_LOG((UB *)"[barcode] ERROR: scanner_init er=%d\n", error);
        }
    }

#if LINE_COMMISSIONING_MODE
    commissioning_phase("white");
    commissioning_phase("black");
#endif

    if (barcode_ready) {
        barcode_current_black = barcode_scanner_read_black();
        segment_start_us = BARCODE_TIMER_RAWL;
        last_edge_us = segment_start_us;
    }

    while (1) {
        uint32_t now_us;

        state = line_update(&raw_bits);
        if (state != last_state) {
            line_reader_format_2bit(raw_bits, bit_text);
            CONSOLE_LOG((UB *)"[line] RL=%s | %s | cmd: %s\n",
                        bit_text, line_state_name(state), line_command(state));
            last_state = state;
        }

        if (barcode_ready) {
            barcode_black = barcode_scanner_read_black();
            now_us = BARCODE_TIMER_RAWL;
            if (barcode_black != barcode_current_black) {
                uint32_t width = now_us - segment_start_us;

                if (barcode_black) {
                    if (barcode_scan_active) {
                        barcode_append_width(barcode_widths,
                                             &barcode_width_count,
                                             &barcode_overflow, width);
                    } else {
                        barcode_scan_active = true;
                        barcode_width_count = 0u;
                        barcode_overflow = false;
                    }
                } else if (barcode_scan_active) {
                    barcode_append_width(barcode_widths,
                                         &barcode_width_count,
                                         &barcode_overflow, width);
                }

                segment_start_us = now_us;
                last_edge_us = now_us;
                barcode_current_black = barcode_black;
            } else if (barcode_scan_active && !barcode_current_black &&
                       (uint32_t)(now_us - last_edge_us) >=
                           BARCODE_SCAN_END_GAP_US) {
                barcode_report_scan(barcode_widths, barcode_width_count,
                                    barcode_overflow);
                barcode_scan_active = false;
                barcode_width_count = 0u;
                barcode_overflow = false;
            }
        }

        heartbeat_ms += LINE_SAMPLE_PERIOD_MS;
        if (heartbeat_ms >= LINE_HEARTBEAT_MS) {
            line_reader_raw_t raw = line_reader_get_last_raw();

            heartbeat_ms = 0u;
            CONSOLE_LOG((UB *)"[line] raw pins: GP%d=%d GP%d=%d\n",
                        (INT)LINE_READER_LEFT_GUARD_GPIO, (INT)raw.left_guard,
                        (INT)LINE_READER_RIGHT_GUARD_GPIO, (INT)raw.right_guard);
        }

        tk_dly_tsk(LINE_SAMPLE_PERIOD_MS);
    }
}
