/* line_logic.h - calibration, line error, line-seen and junction logic.
 * Pure C: no RTOS, no hardware. Unit-tested on a PC (test_buddy3_host.c). */
#ifndef LINE_LOGIC_H
#define LINE_LOGIC_H

#include "buddy3_types.h"

typedef struct {
    uint16_t min_a, max_a, min_b, max_b;
    uint32_t samples;
} line_cal_acc_t;

typedef struct {
    uint16_t white_a, black_a, white_b, black_b;   /* raw ADC counts */
} line_cal_t;

typedef struct {
    uint8_t  junction_count;
    bool     junction_armed;
    uint16_t lost_count;
    bool     line_seen;
} line_track_t;

void         line_cal_begin(line_cal_acc_t *acc);
void         line_cal_add(line_cal_acc_t *acc, uint16_t raw_a, uint16_t raw_b);
cal_result_t line_cal_finish(const line_cal_acc_t *acc, line_cal_t *out);

uint16_t line_normalise(uint16_t raw, uint16_t white, uint16_t black);

void line_track_reset(line_track_t *t);
/* One 1 ms step. Fills *out; sets *junction_event for exactly one step per
 * junction (FR-05). */
void line_track_step(line_track_t *t, const line_cal_t *cal,
                     uint16_t raw_a, uint16_t raw_b,
                     line_state_t *out, bool *junction_event);

#endif /* LINE_LOGIC_H */
