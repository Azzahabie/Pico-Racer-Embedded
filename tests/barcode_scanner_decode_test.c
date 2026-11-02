#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "../app_program/line_task/barcode_scanner.h"

static void append_symbol(const char *pattern, uint32_t narrow,
                          uint32_t *widths, size_t *position)
{
    size_t i;

    for (i = 0u; i < 9u; ++i) {
        widths[*position] = (pattern[i] == 'W') ? narrow * 3u : narrow;
        ++(*position);
    }
}

static void make_a_scan(uint32_t widths[29], uint32_t narrow)
{
    size_t position = 0u;

    append_symbol("NWNNWNWNN", narrow, widths, &position);
    widths[position++] = narrow;
    append_symbol("WNNNNWNNW", narrow, widths, &position);
    widths[position++] = narrow;
    append_symbol("NWNNWNWNN", narrow, widths, &position);
}

static void make_abc_scan(uint32_t widths[49])
{
    size_t position = 0u;

    append_symbol("NWNNWNWNN", 100u, widths, &position);
    widths[position++] = 100u;
    append_symbol("WNNNNWNNW", 100u, widths, &position);
    widths[position++] = 100u;
    append_symbol("NNWNNWNNW", 100u, widths, &position);
    widths[position++] = 100u;
    append_symbol("WNWNNWNNN", 100u, widths, &position);
    widths[position++] = 100u;
    append_symbol("NWNNWNWNN", 100u, widths, &position);
}

static int test_forward_a(void)
{
    uint32_t widths[29];
    barcode_decode_result_t result;

    make_a_scan(widths, 100u);
    barcode_decode_widths(widths, 29u, &result);
    return result.valid && !result.reversed &&
           (result.elements == 9u) && (result.text[0] == 'A') &&
           (result.text[1] == '\0');
}

static int test_reversed_a(void)
{
    uint32_t widths[29];
    uint32_t reversed[29];
    barcode_decode_result_t result;
    size_t i;

    make_a_scan(widths, 100u);
    for (i = 0u; i < 29u; ++i) {
        reversed[i] = widths[28u - i];
    }
    barcode_decode_widths(reversed, 29u, &result);
    return result.valid && result.reversed &&
           (result.elements == 9u) && (result.text[0] == 'A') &&
           (result.text[1] == '\0');
}

static int test_bad_length(void)
{
    uint32_t widths[29];
    barcode_decode_result_t result;

    make_a_scan(widths, 100u);
    barcode_decode_widths(widths, 28u, &result);
    return !result.valid && (result.error == BARCODE_DECODE_BAD_LENGTH);
}

static int test_adaptive_width_scales(void)
{
    static const uint32_t scales[] = { 10u, 37u, 1200u };
    uint32_t widths[29];
    barcode_decode_result_t result;
    size_t i;

    for (i = 0u; i < (sizeof(scales) / sizeof(scales[0])); ++i) {
        make_a_scan(widths, scales[i]);
        barcode_decode_widths(widths, 29u, &result);
        if (!result.valid || (result.text[0] != 'A')) {
            return 0;
        }
    }
    return 1;
}

static int test_complete_alphabet_string(void)
{
    uint32_t widths[49];
    uint32_t reversed[49];
    barcode_decode_result_t result;
    size_t i;

    make_abc_scan(widths);
    barcode_decode_widths(widths, 49u, &result);
    if (!result.valid || result.reversed || (result.elements != 27u) ||
        (result.text[0] != 'A') || (result.text[1] != 'B') ||
        (result.text[2] != 'C') || (result.text[3] != '\0')) {
        return 0;
    }

    for (i = 0u; i < 49u; ++i) {
        reversed[i] = widths[48u - i];
    }
    barcode_decode_widths(reversed, 49u, &result);
    return result.valid && result.reversed &&
           (result.elements == 27u) && (result.text[0] == 'A') &&
           (result.text[1] == 'B') && (result.text[2] == 'C') &&
           (result.text[3] == '\0');
}

int main(void)
{
    if (!test_forward_a()) {
        (void)fprintf(stderr, "forward A decode failed\n");
        return EXIT_FAILURE;
    }
    if (!test_reversed_a()) {
        (void)fprintf(stderr, "reversed A decode failed\n");
        return EXIT_FAILURE;
    }
    if (!test_bad_length()) {
        (void)fprintf(stderr, "bad length was not rejected\n");
        return EXIT_FAILURE;
    }
    if (!test_adaptive_width_scales()) {
        (void)fprintf(stderr, "adaptive width-scale decode failed\n");
        return EXIT_FAILURE;
    }
    if (!test_complete_alphabet_string()) {
        (void)fprintf(stderr, "multi-letter decode failed\n");
        return EXIT_FAILURE;
    }
    (void)puts("barcode scanner decoder: PASS");
    return EXIT_SUCCESS;
}
