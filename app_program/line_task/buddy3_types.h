/* buddy3_types.h - data shared between Buddy 3 modules and other tasks. */
#ifndef BUDDY3_TYPES_H
#define BUDDY3_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "buddy3_config.h"

/* ---- line_state: ir_sense_task -> motion_task (Section 4) ---- */
typedef struct {
    int16_t  error;      /* -1000..+1000, sign from B3_LINE_ERROR_SIGN */
    uint16_t dark_a;     /* 0..1000, sensor on GP26 */
    uint16_t dark_b;     /* 0..1000, sensor on GP27 */
    bool     line_seen;  /* false only after [M] ms below [L_min] */
} line_state_t;

typedef enum {
    CAL_OK = 0,
    CAL_FAIL_NO_DATA,
    CAL_FAIL_RAIL,       /* a reading sat at 0 or full scale */
    CAL_FAIL_CONTRAST    /* black/white contrast below [K_min] */
} cal_result_t;

/* ---- bar_mbf record: 2 bytes = 2-bit type + 14-bit value ---- */
typedef uint16_t bar_rec_t;

typedef enum {
    BAR_REC_BAR   = 0,   /* value = bar width in ms */
    BAR_REC_SPACE = 1,   /* value = space width in ms */
    BAR_REC_END   = 2,   /* quiet gap seen, decode now */
    BAR_REC_ABORT = 3    /* value = abort_reason_t, discard widths */
} bar_rec_type_t;

typedef enum {
    ABORT_NONE     = 0,
    ABORT_TOO_MANY = 1,  /* more than [W_max] widths */
    ABORT_TOO_LONG = 2,  /* a width longer than [W_long] */
    ABORT_OVERFLOW = 3   /* bar_mbf was full, a width was not sent */
} abort_reason_t;

#define BAR_REC_TYPE_SHIFT   14u
#define BAR_REC_VALUE_MASK   0x3FFFu

static inline bar_rec_t bar_rec_pack(bar_rec_type_t type, uint16_t value)
{
    return (bar_rec_t)(((uint16_t)type << BAR_REC_TYPE_SHIFT) |
                       (value & BAR_REC_VALUE_MASK));
}
static inline bar_rec_type_t bar_rec_type(bar_rec_t r)
{
    return (bar_rec_type_t)(r >> BAR_REC_TYPE_SHIFT);
}
static inline uint16_t bar_rec_value(bar_rec_t r)
{
    return (uint16_t)(r & BAR_REC_VALUE_MASK);
}
static inline bool bar_rec_is_end(bar_rec_t r)
{
    return bar_rec_type(r) == BAR_REC_END;
}
static inline bool bar_rec_is_abort(bar_rec_t r)
{
    return bar_rec_type(r) == BAR_REC_ABORT;
}
static inline abort_reason_t bar_rec_abort_reason(bar_rec_t r)
{
    return (abort_reason_t)bar_rec_value(r);
}
static inline uint16_t bar_rec_width_value(bar_rec_t r)
{
    return bar_rec_value(r);
}
static inline bar_rec_t bar_rec_make_abort(abort_reason_t reason)
{
    return bar_rec_pack(BAR_REC_ABORT, (uint16_t)reason);
}

/* ---- barcode commands (FR-07). END_COURSE only if the team picks a
 *      dedicated end barcode (Section 5, Assumption 1). ---- */
typedef enum {
    BC_ACT_NONE = 0,
    BC_ACT_LEFT,
    BC_ACT_RIGHT,
    BC_ACT_STRAIGHT,
    BC_ACT_UTURN,
    BC_ACT_END_COURSE
} barcode_action_t;

typedef enum {
    REJ_NONE = 0,
    REJ_EMPTY,
    REJ_TOO_MANY,    /* from ABORT_TOO_MANY */
    REJ_TOO_LONG,    /* from ABORT_TOO_LONG */
    REJ_OVERFLOW,    /* from ABORT_OVERFLOW */
    REJ_BAD_COUNT,   /* no table entry has this many elements */
    REJ_AMBIGUOUS,   /* a width between narrow and wide */
    REJ_NO_MATCH     /* pattern not in the active table */
} reject_reason_t;

/* ---- active barcode table ----
 * pattern is a string of 'N' and 'W' (bar, space, bar, ... in order). */
typedef struct {
    char    pattern[B3_BARCODE_MAX_ELEMENTS + 1u];
    uint8_t action;      /* barcode_action_t */
} barcode_entry_t;

typedef struct {
    uint8_t         version;   /* reported as mpv= by comms_task */
    uint8_t         count;
    barcode_entry_t entry[B3_BARCODE_MAX_ENTRIES];
} barcode_table_t;

typedef struct {
    bool    valid;
    uint8_t action;
    char    character;  /* decoded Code 39 letter, or '\0' */
    uint8_t reject;      /* reject_reason_t */
    uint8_t count;       /* number of widths decoded */
    char    pattern[B3_BARCODE_MAX_ELEMENTS + 1u];  /* e.g. "NWNNWNWNN",
                                                       '?' = ambiguous */
} barcode_result_t;

#endif /* BUDDY3_TYPES_H */
