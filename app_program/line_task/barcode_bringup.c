/*
 * barcode_bringup.c - stand-alone test of ONE barcode sensor (GP26).
 * No RTOS, no line sensors, no WiFi. Prints over USB serial.
 *
 * How it mirrors the real design (Section 4):
 *   1 ms timer callback  = ir_sense_task tick   (bar_cap_step)
 *   ring buffer          = bar_mbf
 *   main loop            = barcode_task          (barcode_asm_push + decode)
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "../common/messages.h"
#include "bar_capture.h"
#include "bar_ring.h"
#include "barcode_decode.h"

#define IDLE_PRINT_MS      500u

static bar_cap_t g_cap;
static volatile uint32_t g_dropped;

/* ring_put wrapper that also counts drops (runs in the timer interrupt) */
static bool sink(bar_rec_t rec)
{
    bool ok = bar_ring_sink(rec);

    if (!ok) {
        g_dropped++;
    }
    return ok;
}

static bool tick_cb(repeating_timer_t *t)
{
    (void)t;
    bar_cap_step(&g_cap, gpio_get(B3_BARCODE_GPIO), sink);
    return true;
}

int main(void)
{
    barcode_table_t   table;
    barcode_asm_t     assembler;
    barcode_result_t  result;
    repeating_timer_t timer;
    bar_rec_t         rec;
    uint32_t          last_activity_ms;
    uint32_t          accepted = 0u;
    uint32_t          rejected = 0u;
    uint8_t           index = 0u;

    stdio_init_all();
    while (!stdio_usb_connected()) {          /* wait for the serial monitor */
        sleep_ms(100);
    }

    gpio_init(B3_BARCODE_GPIO);
    gpio_set_dir(B3_BARCODE_GPIO, GPIO_IN);

    barcode_table_load_default(&table);
    barcode_asm_reset(&assembler);
    bar_cap_reset(&g_cap);
    bar_ring_reset(&g_bar_ring);

    printf("\nCode 39 single-letter reader (*A* through *Z*). "
           "GP%u, bar level=%d, Q=%u ms, W_max=%u\n",
           B3_BARCODE_GPIO, (int)B3_BARCODE_BAR_LEVEL,
           (unsigned)B3_BAR_GAP_END_MS, (unsigned)B3_BAR_MAX_WIDTHS);

    add_repeating_timer_ms(-1, tick_cb, NULL, &timer);   /* every 1 ms */
    last_activity_ms = to_ms_since_boot(get_absolute_time());

    for (;;) {
        if (bar_ring_get(&g_bar_ring, &rec)) {
            last_activity_ms = to_ms_since_boot(get_absolute_time());

            if (bar_rec_is_end(rec)) {
                printf("END\n");
            } else if (bar_rec_is_abort(rec)) {
                printf("ABORT reason=%u\n", (unsigned)bar_rec_abort_reason(rec));
            } else {
                /* bar or space is implied by order: first width is a bar */
                printf("%s %u\n", ((index & 1u) == 0u) ? "BAR  " : "SPACE",
                       (unsigned)bar_rec_width_value(rec));
            }
            index = (bar_rec_is_end(rec) || bar_rec_is_abort(rec)) ? 0u : (uint8_t)(index + 1u);

            if (barcode_asm_push(&assembler, rec, &table, &result)) {
                if (result.valid != 0u) {
                    accepted++;
                    if (result.character != '\0') {
                        printf(">> OK     %s -> LETTER %c   (ok=%lu rej=%lu)\n\n",
                               result.pattern, result.character,
                               (unsigned long)accepted, (unsigned long)rejected);
                    } else {
                        printf(">> OK     %s -> %s   (ok=%lu rej=%lu)\n\n",
                               result.pattern,
                               barcode_action_name((uint8_t)result.action),
                               (unsigned long)accepted, (unsigned long)rejected);
                    }
                } else {
                    rejected++;
                    printf(">> REJECT %s  pattern=%s n=%u   (ok=%lu rej=%lu)\n\n",
                           barcode_reject_name((uint8_t)result.reject), result.pattern,
                           (unsigned)result.count,
                           (unsigned long)accepted, (unsigned long)rejected);
                }
            }
        } else if ((to_ms_since_boot(get_absolute_time()) - last_activity_ms)
                   >= IDLE_PRINT_MS) {
            /* quiet: show the raw level so polarity can be checked by hand */
            printf("idle: GP%u=%d (%s)  dropped=%lu\n", B3_BARCODE_GPIO,
                   (int)gpio_get(B3_BARCODE_GPIO),
                   (gpio_get(B3_BARCODE_GPIO) == (B3_BARCODE_BAR_LEVEL != 0))
                       ? "reads BAR" : "reads SPACE",
                   (unsigned long)g_dropped);
            last_activity_ms = to_ms_since_boot(get_absolute_time());
        }
    }
}
