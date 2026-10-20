#include <string.h>
#include "barcode_decode.h"

static uint16_t code39_pattern_mask(const char *pattern)
{
    uint16_t mask = 0u;
    uint8_t i;

    for (i = 0u; i < 9u; i++) {
        mask = (uint16_t)(mask << 1);
        if (pattern[i] == 'W') {
            mask |= 1u;
        }
    }
    return mask;
}

static bool code39_letter(const char *pattern, char *letter)
{
    static const uint8_t bar_weights[5] = { 1u, 2u, 4u, 7u, 0u };
    uint8_t bar_value = 0u;
    uint8_t wide_bars = 0u;
    uint8_t wide_spaces = 0u;
    uint8_t wide_space_index = 0u;
    uint8_t letter_rank;
    uint8_t i;
    uint8_t alphabet_index;

    for (i = 0u; i < 9u; i++) {
        if (pattern[i] != 'W') {
            continue;
        }
        if ((i & 1u) == 0u) {
            bar_value = (uint8_t)(bar_value + bar_weights[i / 2u]);
            wide_bars++;
        } else {
            wide_spaces++;
            wide_space_index = (uint8_t)(i / 2u);
        }
    }

    if ((wide_bars != 2u) || (wide_spaces != 1u) ||
        (bar_value == 0u) || (bar_value == 10u) || (bar_value > 11u)) {
        return false;
    }

    letter_rank = (bar_value == 11u) ? 10u : bar_value;
    switch (wide_space_index) {
    case 0u: /* U-Z */
        if (letter_rank > 6u) {
            return false;
        }
        alphabet_index = (uint8_t)(20u + letter_rank);
        break;
    case 2u: /* A-J */
        alphabet_index = letter_rank;
        break;
    case 3u: /* K-T */
        alphabet_index = (uint8_t)(10u + letter_rank);
        break;
    default:
        return false;
    }

    *letter = (char)('A' + alphabet_index - 1u);
    return true;
}

void barcode_table_load_default(barcode_table_t *t)
{
    memset(t, 0, sizeof(*t));
}

bool barcode_table_validate(const barcode_table_t *t)
{
    uint8_t i;
    uint8_t j;

    if ((t->count == 0u) || (t->count > B3_BARCODE_MAX_ENTRIES)) {
        return false;
    }
    for (i = 0u; i < t->count; i++) {
        size_t len = 0u;

        while ((len <= B3_BARCODE_MAX_ELEMENTS) &&
               (t->entry[i].pattern[len] != '\0')) {
            len++;
        }
        if ((len == 0u) || (len > B3_BARCODE_MAX_ELEMENTS) ||
            (t->entry[i].action == (uint8_t)BC_ACT_NONE) ||
            (t->entry[i].action > (uint8_t)BC_ACT_END_COURSE)) {
            return false;
        }
        for (size_t k = 0u; k < len; k++) {
            if ((t->entry[i].pattern[k] != 'N') &&
                (t->entry[i].pattern[k] != 'W')) {
                return false;
            }
        }
        for (j = 0u; j < i; j++) {
            if (strcmp(t->entry[j].pattern, t->entry[i].pattern) == 0) {
                return false;
            }
        }
    }
    return true;
}

void barcode_decode(const barcode_table_t *t, const uint16_t *w,
                    uint8_t n, barcode_result_t *r)
{
    uint8_t  i;
    uint16_t ref[2];
    bool     ambiguous = false;

    r->pattern[0] = '\0';
    r->valid = false;
    r->action = (uint8_t)BC_ACT_NONE;
    r->character = '\0';
    r->reject = (uint8_t)REJ_NONE;
    r->count = n;

    if (n == 0u) {
        r->reject = (uint16_t)REJ_EMPTY;
        return;
    }
    if (n > B3_BAR_MAX_WIDTHS) {
        r->reject = (uint16_t)REJ_TOO_MANY;
        return;
    }
    /* reference width = shortest width in THIS barcode, so the result does
     * not depend on speed (A7). Bars and spaces get their own reference
     * (B3_RATIO_SEPARATE_BAR_SPACE) to cancel threshold / blur bias. */
    ref[0] = UINT16_MAX;                /* bars   = even index */
    ref[1] = UINT16_MAX;                /* spaces = odd index  */
    for (i = 0u; i < n; i++) {
        uint8_t k = (uint8_t)(i & 1u);

        if (w[i] < ref[k]) {
            ref[k] = w[i];
        }
    }
    if (ref[1] == UINT16_MAX) {
        ref[1] = ref[0];                /* single element: no spaces */
    }
#if !B3_RATIO_SEPARATE_BAR_SPACE
    if (ref[1] < ref[0]) { ref[0] = ref[1]; }
    ref[1] = ref[0];
#endif
    if ((ref[0] == 0u) || (ref[1] == 0u)) {
        r->reject = (uint16_t)REJ_AMBIGUOUS;
        return;
    }

    for (i = 0u; i < n; i++) {
        uint32_t ratio = ((uint32_t)w[i] * 100u) / ref[i & 1u];

        if (ratio <= B3_RATIO_NARROW_MAX_X100) {
            r->pattern[i] = 'N';
        } else if (ratio >= B3_RATIO_WIDE_MIN_X100) {
            r->pattern[i] = 'W';
        } else {
            r->pattern[i] = '?';
            ambiguous = true;
        }
    }
    r->pattern[n] = '\0';
    if (ambiguous) {
        r->reject = (uint16_t)REJ_AMBIGUOUS;
        return;
    }

    /* One Code 39 letter is framed by '*' start/stop symbols. */
    if (n == 29u) {
        char decoded;

        if ((r->pattern[9] == 'N') && (r->pattern[19] == 'N') &&
            (code39_pattern_mask(&r->pattern[0]) == 0x094u) &&
            (code39_pattern_mask(&r->pattern[20]) == 0x094u) &&
            code39_letter(&r->pattern[10], &decoded)) {
            r->valid = true;
            r->character = decoded;
            return;
        }
    }

    if ((t->count == 0u) || (t->count > B3_BARCODE_MAX_ENTRIES)) {
        r->reject = (uint8_t)REJ_NO_MATCH;
        return;
    }
    for (i = 0u; i < t->count; i++) {
        if (strcmp(t->entry[i].pattern, r->pattern) == 0) {
            r->valid = true;
            r->action = t->entry[i].action;
            return;
        }
    }
    for (i = 0u; i < t->count; i++) {
        size_t len = 0u;

        while ((len <= B3_BARCODE_MAX_ELEMENTS) &&
               (t->entry[i].pattern[len] != '\0')) {
            len++;
        }
        if (len == n) {
            r->reject = (uint8_t)REJ_NO_MATCH;
            return;
        }
    }
    r->reject = (uint8_t)REJ_BAD_COUNT;
}

const char *barcode_action_name(uint8_t action)
{
    switch ((barcode_action_t)action) {
    case BC_ACT_LEFT:       return "LEFT";
    case BC_ACT_RIGHT:      return "RIGHT";
    case BC_ACT_STRAIGHT:   return "STRAIGHT";
    case BC_ACT_UTURN:      return "UTURN";
    case BC_ACT_END_COURSE: return "END_COURSE";
    default:                return "NONE";
    }
}

const char *barcode_reject_name(uint8_t reject)
{
    switch ((reject_reason_t)reject) {
    case REJ_EMPTY:     return "EMPTY";
    case REJ_TOO_MANY:  return "TOO_MANY_WIDTHS";
    case REJ_TOO_LONG:  return "WIDTH_TOO_LONG";
    case REJ_OVERFLOW:  return "BUFFER_OVERFLOW";
    case REJ_BAD_COUNT: return "BAD_ELEMENT_COUNT";
    case REJ_AMBIGUOUS: return "AMBIGUOUS_WIDTH";
    case REJ_NO_MATCH:  return "NO_TABLE_MATCH";
    default:            return "NONE";
    }
}

void barcode_asm_reset(barcode_asm_t *a)
{
    a->n = 0u;
    a->overflow = 0u;
}

static void fill_reject(barcode_result_t *r, reject_reason_t why, uint8_t n)
{
    r->pattern[0] = '\0';
    r->valid = false;
    r->action = (uint8_t)BC_ACT_NONE;
    r->character = '\0';
    r->reject = (uint8_t)why;
    r->count = n;
}

bool barcode_asm_push(barcode_asm_t *a, bar_rec_t rec,
                      const barcode_table_t *t, barcode_result_t *r)
{
    if (bar_rec_is_end(rec)) {
        if (a->overflow != 0u) {
            fill_reject(r, REJ_TOO_MANY, a->n);
        } else {
            barcode_decode(t, a->w, a->n, r);
        }
        barcode_asm_reset(a);
        return true;
    }

    if (bar_rec_is_abort(rec)) {
        switch (bar_rec_abort_reason(rec)) {
        case ABORT_TOO_MANY: fill_reject(r, REJ_TOO_MANY, a->n); break;
        case ABORT_TOO_LONG: fill_reject(r, REJ_TOO_LONG, a->n); break;
        default:             fill_reject(r, REJ_OVERFLOW, a->n); break;
        }
        barcode_asm_reset(a);
        return true;
    }

    /* a width: the first is a bar, then they alternate */
    if (a->n < B3_BAR_MAX_WIDTHS) {
        if ((bar_rec_type(rec) != BAR_REC_BAR) &&
            (bar_rec_type(rec) != BAR_REC_SPACE)) {
            return false;
        }
        a->w[a->n] = bar_rec_width_value(rec);
        a->n++;
    } else {
        a->overflow = 1u;
    }
    return false;
}
