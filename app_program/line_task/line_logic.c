#include "line_logic.h"

static bool at_rail(uint16_t v)
{
    return (v <= B3_ADC_RAIL_MARGIN) ||
           (v >= (B3_ADC_MAX - B3_ADC_RAIL_MARGIN));
}

void line_cal_begin(line_cal_acc_t *acc)
{
    acc->min_a = 0xFFFFu;
    acc->max_a = 0u;
    acc->min_b = 0xFFFFu;
    acc->max_b = 0u;
    acc->samples = 0u;
}

void line_cal_add(line_cal_acc_t *acc, uint16_t raw_a, uint16_t raw_b)
{
    if (raw_a < acc->min_a) { acc->min_a = raw_a; }
    if (raw_a > acc->max_a) { acc->max_a = raw_a; }
    if (raw_b < acc->min_b) { acc->min_b = raw_b; }
    if (raw_b > acc->max_b) { acc->max_b = raw_b; }
    acc->samples++;
}

cal_result_t line_cal_finish(const line_cal_acc_t *acc, line_cal_t *out)
{
    if (acc->samples == 0u) {
        return CAL_FAIL_NO_DATA;
    }
    if (at_rail(acc->min_a) || at_rail(acc->max_a) ||
        at_rail(acc->min_b) || at_rail(acc->max_b)) {
        return CAL_FAIL_RAIL;
    }
    if (((uint16_t)(acc->max_a - acc->min_a) < B3_CAL_MIN_CONTRAST) ||
        ((uint16_t)(acc->max_b - acc->min_b) < B3_CAL_MIN_CONTRAST)) {
        return CAL_FAIL_CONTRAST;
    }
#if B3_LINE_DARK_IS_HIGH
    out->black_a = acc->max_a;  out->white_a = acc->min_a;
    out->black_b = acc->max_b;  out->white_b = acc->min_b;
#else
    out->black_a = acc->min_a;  out->white_a = acc->max_a;
    out->black_b = acc->min_b;  out->white_b = acc->max_b;
#endif
    return CAL_OK;
}

uint16_t line_normalise(uint16_t raw, uint16_t white, uint16_t black)
{
    int32_t num = ((int32_t)raw - (int32_t)white) * (int32_t)B3_NORM_FULL;
    int32_t den = (int32_t)black - (int32_t)white;
    int32_t v;

    if (den == 0) {
        return 0u;
    }
    v = num / den;
    if (v < 0) {
        v = 0;
    }
    if (v > (int32_t)B3_NORM_FULL) {
        v = (int32_t)B3_NORM_FULL;
    }
    return (uint16_t)v;
}

void line_track_reset(line_track_t *t)
{
    t->junction_count = 0u;
    t->junction_armed = true;
    t->lost_count = 0u;
    t->line_seen = true;   /* assume on the line when a mission starts */
}

void line_track_step(line_track_t *t, const line_cal_t *cal,
                     uint16_t raw_a, uint16_t raw_b,
                     line_state_t *out, bool *junction_event)
{
    uint16_t a = line_normalise(raw_a, cal->white_a, cal->black_a);
    uint16_t b = line_normalise(raw_b, cal->white_b, cal->black_b);
    int32_t  err = ((int32_t)b - (int32_t)a) * (int32_t)B3_LINE_ERROR_SIGN;
    bool     both_dark = (a >= B3_JUNCTION_DARK) && (b >= B3_JUNCTION_DARK);

    *junction_event = false;

    /* line seen: drops only after [M] ms below [L_min] (FR-03) */
    if ((uint16_t)(a + b) >= B3_LINE_MIN_SIGNAL) {
        t->lost_count = 0u;
        t->line_seen = true;
    } else {
        if (t->lost_count < 0xFFFFu) {
            t->lost_count++;
        }
        if (t->lost_count >= B3_LINE_LOST_MS) {
            t->line_seen = false;
        }
    }

    /* junction: both dark for [J_n] samples, one event until released */
    if (both_dark) {
        if (t->junction_count < 0xFFu) {
            t->junction_count++;
        }
        if ((t->junction_count >= B3_JUNCTION_DEBOUNCE) && t->junction_armed) {
            *junction_event = true;
            t->junction_armed = false;
        }
    } else {
        t->junction_count = 0u;
        if ((a < B3_JUNCTION_RELEASE) || (b < B3_JUNCTION_RELEASE)) {
            t->junction_armed = true;
        }
    }

    out->error = (int16_t)err;
    out->dark_a = a;
    out->dark_b = b;
    out->line_seen = t->line_seen;
}
