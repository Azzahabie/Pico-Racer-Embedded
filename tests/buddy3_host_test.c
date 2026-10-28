#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../app_program/line_task/bar_capture.h"
#include "../app_program/line_task/bar_ring.h"
#include "../app_program/line_task/barcode_decode.h"
#include "../app_program/line_task/line_logic.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return false; \
    } \
} while (0)

static const char *const letter_patterns[26] = {
    "WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",
    "WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",
    "NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",
    "WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",
    "NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",
    "WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",
    "WWNNWNNNN", "NWWNWNNNN"
};

static void append_symbol(const char *symbol, uint16_t narrow,
                          uint16_t widths[29], uint8_t *position)
{
    uint8_t i;

    for (i = 0u; i < 9u; i++) {
        widths[*position] = (symbol[i] == 'W')
                          ? (uint16_t)(narrow * 3u) : narrow;
        (*position)++;
    }
}

static void make_capture(char letter, uint16_t narrow, uint16_t widths[29])
{
    uint8_t position = 0u;

    append_symbol("NWNNWNWNN", narrow, widths, &position);
    widths[position++] = narrow;
    append_symbol(letter_patterns[(uint8_t)(letter - 'A')], narrow,
                  widths, &position);
    widths[position++] = narrow;
    append_symbol("NWNNWNWNN", narrow, widths, &position);
}

static bool test_all_code39_letters(void)
{
    uint16_t widths[29];
    barcode_result_t result;
    uint8_t i;

    for (i = 0u; i < 26u; i++) {
        make_capture((char)('A' + i), 10u, widths);
        barcode_decode(widths, 29u, &result);
        if (!result.valid) {
            (void)fprintf(stderr, "decode %c rejected %s: %s\n",
                          (char)('A' + i),
                          barcode_reject_name(result.reject), result.pattern);
        }
        CHECK(result.valid);
        CHECK(result.character == (char)('A' + i));
        CHECK(result.action == (uint8_t)BC_ACT_NONE);
    }
    return true;
}

static bool test_scale_and_noise(void)
{
    static const uint16_t scales[] = { 3u, 17u, 200u };
    uint16_t widths[29];
    barcode_result_t result;
    uint8_t s;
    uint8_t i;

    for (s = 0u; s < (uint8_t)(sizeof(scales) / sizeof(scales[0])); s++) {
        make_capture('X', scales[s], widths);
        barcode_decode(widths, 29u, &result);
        CHECK(result.valid && result.character == 'X');
    }

    make_capture('F', 100u, widths);
    for (i = 0u; i < 29u; i++) {
        uint16_t adjustment = (uint16_t)((i % 5u) + 1u);
        if ((i & 1u) != 0u) {
            widths[i] += adjustment;
        } else {
            widths[i] -= adjustment;
        }
    }
    barcode_decode(widths, 29u, &result);
    CHECK(result.valid && result.character == 'F');

    make_capture('A', 100u, widths);
    widths[11] = 5u;
    barcode_decode(widths, 29u, &result);
    CHECK(result.valid && result.character == 'A');
    return true;
}

static bool test_decoder_rejections(void)
{
    uint16_t widths[33];
    uint16_t reversed[29];
    barcode_result_t result;
    uint8_t i;

    make_capture('A', 10u, widths);
    barcode_decode(widths, 0u, &result);
    CHECK(result.reject == (uint8_t)REJ_EMPTY);
    barcode_decode(widths, 28u, &result);
    CHECK(result.reject == (uint8_t)REJ_BAD_COUNT);
    barcode_decode(widths, 33u, &result);
    CHECK(result.reject == (uint8_t)REJ_TOO_MANY);

    make_capture('A', 10u, widths);
    widths[1] = 10u;
    barcode_decode(widths, 29u, &result);
    CHECK(!result.valid);

    make_capture('A', 10u, widths);
    widths[21] = 10u;
    barcode_decode(widths, 29u, &result);
    CHECK(result.reject == (uint8_t)REJ_BAD_FRAME);

    make_capture('A', 10u, widths);
    /* Three wide spaces is not a standard Code 39 letter. */
    {
        uint8_t pos = 10u;
        append_symbol("NWNWNWNNN", 10u, widths, &pos);
    }
    barcode_decode(widths, 29u, &result);
    CHECK(result.reject == (uint8_t)REJ_UNKNOWN_PATTERN);

    make_capture('A', 10u, widths);
    for (i = 0u; i < 29u; i++) {
        reversed[i] = widths[28u - i];
    }
    barcode_decode(reversed, 29u, &result);
    CHECK(!result.valid);
    CHECK(result.reject == (uint8_t)REJ_UNKNOWN_PATTERN ||
          result.reject == (uint8_t)REJ_REVERSED ||
          result.reject == (uint8_t)REJ_BAD_FRAME);

    make_capture('A', 10u, widths);
    widths[0] = 10u;
    widths[1] = 30u;
    widths[2] = 10u;
    widths[3] = 30u;
    widths[4] = 10u;
    widths[5] = 30u;
    widths[6] = 10u;
    widths[7] = 30u;
    widths[8] = 10u;
    barcode_decode(widths, 29u, &result);
    CHECK(!result.valid);
    return true;
}

static bool test_assembler_recovery(void)
{
    barcode_asm_t assembler;
    barcode_result_t result;
    uint16_t widths[29];
    uint8_t i;

    make_capture('Z', 10u, widths);
    barcode_asm_reset(&assembler);
    CHECK(barcode_asm_push(&assembler,
          bar_rec_pack(BAR_REC_BAR, 10u), &result) == false);
    CHECK(barcode_asm_push(&assembler,
          bar_rec_pack(BAR_REC_END, 0u), &result) == true);
    CHECK(result.reject == (uint8_t)REJ_BAD_COUNT);

    for (i = 0u; i < 29u; i++) {
        bar_rec_type_t type = ((i & 1u) == 0u) ? BAR_REC_BAR : BAR_REC_SPACE;
        CHECK(barcode_asm_push(&assembler,
              bar_rec_pack(type, widths[i]), &result) == false);
    }
    CHECK(barcode_asm_push(&assembler,
          bar_rec_pack(BAR_REC_END, 0u), &result) == true);
    CHECK(result.valid && result.character == 'Z');

    CHECK(barcode_asm_push(&assembler,
          bar_rec_make_abort(ABORT_TOO_LONG), &result) == true);
    CHECK(result.reject == (uint8_t)REJ_TOO_LONG);
    CHECK(barcode_asm_push(&assembler,
          bar_rec_make_abort(ABORT_OVERFLOW), &result) == true);
    CHECK(result.reject == (uint8_t)REJ_OVERFLOW);

    for (i = 0u; i < (B3_BAR_MAX_WIDTHS + 1u); i++) {
        bar_rec_type_t type = ((i & 1u) == 0u) ? BAR_REC_BAR : BAR_REC_SPACE;
        (void)barcode_asm_push(&assembler,
              bar_rec_pack(type, 10u), &result);
    }
    CHECK(barcode_asm_push(&assembler,
          bar_rec_pack(BAR_REC_END, 0u), &result) == true);
    CHECK(result.reject == (uint8_t)REJ_TOO_MANY);
    return true;
}

static bool test_line_truth_table(void)
{
    static const uint8_t expected_bits[4] = { 0u, 1u, 2u, 3u };
    line_track_t tracker;
    line_state_t state;
    bool dark_is_high;
    uint8_t raw;

    for (dark_is_high = false; ; dark_is_high = true) {
        for (raw = 0u; raw < 4u; raw++) {
            bool a_high = ((raw & 1u) != 0u) ? dark_is_high : !dark_is_high;
            bool b_high = ((raw & 2u) != 0u) ? dark_is_high : !dark_is_high;
            line_track_reset(&tracker);
            line_track_step(&tracker, a_high, b_high, dark_is_high, &state);
            CHECK(state.sensor_bits == expected_bits[raw]);
            CHECK(state.line_seen == (raw != 0u));
            if (raw == 0u) {
                CHECK(state.observation == LINE_OBS_LOST);
                CHECK(state.steering == LINE_STEER_STOP);
            } else if (raw == 1u) {
                CHECK(state.observation == LINE_OBS_SENSOR_A);
                CHECK(state.steering == LINE_STEER_LEFT);
            } else if (raw == 2u) {
                CHECK(state.observation == LINE_OBS_SENSOR_B);
                CHECK(state.steering == LINE_STEER_RIGHT);
            } else {
                CHECK(state.observation == LINE_OBS_BOTH);
                CHECK(state.steering == LINE_STEER_STRAIGHT);
            }
        }
        if (dark_is_high) {
            break;
        }
    }
    return true;
}

static bool test_junction_debounce_and_rearm(void)
{
    line_track_t tracker;
    line_state_t state;
    uint8_t i;

    line_track_reset(&tracker);
    line_track_step(&tracker, false, false, true, &state);
    for (i = 0u; i < 4u; i++) {
        line_track_step(&tracker, true, true, true, &state);
        if (i < 3u) {
            CHECK(!state.junction_event);
        }
    }
    CHECK(state.junction_event);
    line_track_step(&tracker, true, true, true, &state);
    CHECK(!state.junction_event);

    line_track_step(&tracker, false, true, true, &state);
    line_track_step(&tracker, false, true, true, &state);
    line_track_step(&tracker, true, true, true, &state);
    line_track_step(&tracker, true, true, true, &state);
    line_track_step(&tracker, true, true, true, &state);
    line_track_step(&tracker, true, true, true, &state);
    CHECK(state.junction_event);
    return true;
}

static bool test_ring_buffer(void)
{
    bar_rec_t record;
    uint8_t accepted = 0u;
    uint8_t i;

    bar_ring_reset(&g_bar_ring);
    for (i = 0u; i < B3_BAR_MBF_RECORDS; i++) {
        if (!bar_ring_sink(bar_rec_pack(BAR_REC_BAR, i))) {
            break;
        }
        accepted++;
    }
    CHECK(accepted == (B3_BAR_MBF_RECORDS - 1u));
    CHECK(!bar_ring_sink(bar_rec_pack(BAR_REC_END, 0u)));
    CHECK(bar_ring_get(&g_bar_ring, &record));
    CHECK(bar_ring_sink(bar_rec_pack(BAR_REC_END, 0u)));
    while (bar_ring_get(&g_bar_ring, &record)) {
    }
    CHECK(!bar_ring_get(&g_bar_ring, &record));
    return true;
}

static uint8_t capture_sink_failures;

static bool capture_sink(bar_rec_t record)
{
    if (capture_sink_failures > 0u) {
        capture_sink_failures--;
        return false;
    }
    return bar_ring_sink(record);
}

static void capture_level(bar_cap_t *capture, bool is_bar)
{
    bool level = is_bar ? (B3_BARCODE_BAR_LEVEL != 0u)
                        : (B3_BARCODE_BAR_LEVEL == 0u);
    bar_cap_step(capture, level, capture_sink);
}

static void feed_capture(bar_cap_t *capture, const uint16_t widths[29],
                         bool inject_glitch)
{
    uint16_t tick;
    uint8_t i;

    for (tick = 0u; tick < 4u; tick++) {
        capture_level(capture, false);
    }
    for (i = 0u; i < 29u; i++) {
        bool is_bar = ((i & 1u) == 0u);
        for (tick = 0u; tick < widths[i]; tick++) {
            bool glitch = inject_glitch && (i == 2u) && (tick == 5u);
            capture_level(capture, glitch ? !is_bar : is_bar);
        }
    }
    for (tick = 0u; tick < B3_BAR_GAP_END_TICKS + 4u; tick++) {
        capture_level(capture, false);
    }
}

static bool test_capture_timeout_and_recovery(void)
{
    bar_cap_t capture;
    barcode_asm_t assembler;
    barcode_result_t result;
    uint16_t widths[29];
    bar_rec_t record;
    bool complete = false;
    uint16_t tick;

    bar_ring_reset(&g_bar_ring);
    bar_cap_reset(&capture);
    barcode_asm_reset(&assembler);
    make_capture('F', 10u, widths);
    feed_capture(&capture, widths, true);
    while (bar_ring_get(&g_bar_ring, &record)) {
        if (barcode_asm_push(&assembler, record, &result)) {
            complete = true;
        }
    }
    CHECK(complete);
    CHECK(result.valid && result.character == 'F');

    /* A capture interrupted by a stuck dark input aborts; the quiet period
     * returns the state machine to idle before the next scan. */
    bar_ring_reset(&g_bar_ring);
    bar_cap_reset(&capture);
    capture_level(&capture, false);
    capture_level(&capture, false);
    capture_level(&capture, true);
    capture_level(&capture, true);
    for (tick = 0u; tick < B3_BAR_LONG_TICKS + 3u; tick++) {
        capture_level(&capture, true);
    }
    CHECK(bar_ring_get(&g_bar_ring, &record));
    CHECK(bar_rec_is_abort(record));
    CHECK(bar_rec_abort_reason(record) == ABORT_TOO_LONG);
    for (tick = 0u; tick < B3_BAR_GAP_END_TICKS + 4u; tick++) {
        capture_level(&capture, false);
    }
    CHECK(capture.state == BAR_CAP_IDLE);

    bar_ring_reset(&g_bar_ring);
    barcode_asm_reset(&assembler);
    make_capture('Y', 10u, widths);
    feed_capture(&capture, widths, false);
    complete = false;
    while (bar_ring_get(&g_bar_ring, &record)) {
        if (barcode_asm_push(&assembler, record, &result)) {
            complete = true;
        }
    }
    CHECK(complete && result.valid && result.character == 'Y');

    bar_ring_reset(&g_bar_ring);
    bar_cap_reset(&capture);
    capture_sink_failures = 1u;
    make_capture('A', 10u, widths);
    feed_capture(&capture, widths, false);
    CHECK(bar_ring_get(&g_bar_ring, &record));
    CHECK(bar_rec_is_abort(record));
    CHECK(bar_rec_abort_reason(record) == ABORT_OVERFLOW);
    return true;
}

static bool run(const char *name, bool (*test)(void))
{
    bool passed = test();
    (void)printf("%s: %s\n", name, passed ? "PASS" : "FAIL");
    return passed;
}

int main(void)
{
    unsigned failures = 0u;

    failures += !run("Code 39 A-Z", test_all_code39_letters);
    failures += !run("Scale and noise", test_scale_and_noise);
    failures += !run("Decoder rejections", test_decoder_rejections);
    failures += !run("Assembler recovery", test_assembler_recovery);
    failures += !run("Line truth table/polarity", test_line_truth_table);
    failures += !run("Junction debounce/rearm", test_junction_debounce_and_rearm);
    failures += !run("Ring overflow/recovery", test_ring_buffer);
    failures += !run("Capture timeout/recovery", test_capture_timeout_and_recovery);
    return failures == 0u ? EXIT_SUCCESS : EXIT_FAILURE;
}
