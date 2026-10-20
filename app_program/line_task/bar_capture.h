/* bar_capture.h - barcode edge capture (6.3 Region B, Capturing).
 * Called once per 1 ms with the raw GP26 level. Sends one 2-byte record per
 * width, then END (quiet gap) or ABORT (sanity limit / ring full).
 * Pure C: the sink is a function pointer. */
#ifndef BAR_CAPTURE_H
#define BAR_CAPTURE_H

#include "buddy3_types.h"

/* Return true if the record was accepted, false if the ring is full. */
typedef bool (*bar_sink_fn)(bar_rec_t rec);

typedef enum {
    BAR_CAP_IDLE = 0,    /* waiting for the first bar edge */
    BAR_CAP_CAPTURING,   /* measuring widths */
    BAR_CAP_DRAIN        /* after ABORT: wait for a quiet gap */
} bar_cap_state_t;

/* since_edge saturates just above the configured long-width limit. */
typedef struct {
    bar_cap_state_t state;
    uint16_t since_edge;
    uint16_t width_count;
    uint8_t deb_count;
    uint8_t abort_reason;
    bool initialised;
    bool stable_is_bar;
    bool pending_abort;
} bar_cap_t;

void bar_cap_reset(bar_cap_t *c);
void bar_cap_step(bar_cap_t *c, bool level_high, bar_sink_fn sink);

#endif /* BAR_CAPTURE_H */
