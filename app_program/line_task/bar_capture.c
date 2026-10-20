#include "bar_capture.h"

#define SINCE_MAX  (B3_BAR_LONG_MS + 1u)

void bar_cap_reset(bar_cap_t *c)
{
    c->state = BAR_CAP_IDLE;
    c->initialised = false;
    c->stable_is_bar = false;
    c->deb_count = 0u;
    c->pending_abort = false;
    c->abort_reason = (uint8_t)ABORT_NONE;
    c->width_count = 0u;
    c->since_edge = 0u;
}

/* Try to send ABORT. If the ring is full keep retrying every tick: a width
 * is never lost silently (report Section 4). */
static void raise_abort(bar_cap_t *c, bar_sink_fn sink, abort_reason_t reason)
{
    c->abort_reason = (uint8_t)reason;
    if (sink(bar_rec_make_abort(reason))) {
        c->pending_abort = false;
        c->state = BAR_CAP_DRAIN;
        c->since_edge = 0u;
    } else {
        c->pending_abort = true;
    }
}

static void capturing(bar_cap_t *c, bool edge, bar_sink_fn sink)
{
    uint32_t elapsed = c->since_edge;

    if (c->pending_abort) {
        raise_abort(c, sink, (abort_reason_t)c->abort_reason);
        return;
    }

    if (edge) {
        if (c->width_count >= B3_BAR_MAX_WIDTHS) {
            raise_abort(c, sink, ABORT_TOO_MANY);
        } else if (elapsed > B3_BAR_LONG_MS) {
            raise_abort(c, sink, ABORT_TOO_LONG);
        } else if (sink(bar_rec_pack((c->width_count & 1u) == 0u
                                     ? BAR_REC_BAR : BAR_REC_SPACE,
                                     (uint16_t)elapsed))) {
            c->width_count++;
            c->since_edge = 0u;
        } else {
            raise_abort(c, sink, ABORT_OVERFLOW);
        }
    } else if (c->stable_is_bar == 0u) {
        /* in a space: a long enough gap is the quiet zone = end */
        if (elapsed >= B3_BAR_GAP_END_MS) {
            if (sink(bar_rec_pack(BAR_REC_END, 0u))) {
                c->state = BAR_CAP_IDLE;
            } else {
                raise_abort(c, sink, ABORT_OVERFLOW);
            }
        }
    } else if (elapsed > B3_BAR_LONG_MS) {
        raise_abort(c, sink, ABORT_TOO_LONG);       /* stuck on black */
    }
}

void bar_cap_step(bar_cap_t *c, bool level_high, bar_sink_fn sink)
{
    bool is_bar = (level_high == (B3_BARCODE_BAR_LEVEL != 0));
    bool edge = false;

    if (c->since_edge < SINCE_MAX) {
        c->since_edge++;
    }

    if (!c->initialised) {                    /* adopt the first level */
        c->stable_is_bar = is_bar;
        c->initialised = true;
        c->since_edge = 0u;
        return;
    }

    if (is_bar == (c->stable_is_bar != 0u)) {
        c->deb_count = 0u;
    } else {
        c->deb_count++;
        if (c->deb_count >= B3_BAR_DEBOUNCE_SAMPLES) {
            c->stable_is_bar = is_bar;
            c->deb_count = 0u;
            edge = true;
        }
    }

    switch ((bar_cap_state_t)c->state) {
    case BAR_CAP_IDLE:
        if (edge && c->stable_is_bar) {       /* first bar starts a barcode */
            c->state = BAR_CAP_CAPTURING;
            c->since_edge = 0u;
            c->width_count = 0u;
            c->pending_abort = false;
        }
        break;
    case BAR_CAP_CAPTURING:
        capturing(c, edge, sink);
        break;
    default:                                       /* BAR_CAP_DRAIN */
        if (edge) {
            c->since_edge = 0u;
        } else if (!c->stable_is_bar && (c->since_edge >= B3_BAR_GAP_END_MS)) {
            c->state = BAR_CAP_IDLE;
        }
        break;
    }
}
