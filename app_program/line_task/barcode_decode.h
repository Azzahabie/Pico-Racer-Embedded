/* barcode_decode.h - narrow/wide classification, single-letter Code 39
 * decoding (*A* through *Z*) and the record assembler. Pure C.
 */
#ifndef BARCODE_DECODE_H
#define BARCODE_DECODE_H

#include "buddy3_types.h"

void barcode_table_load_default(barcode_table_t *t);
bool barcode_table_validate(const barcode_table_t *t);

void barcode_decode(const barcode_table_t *t, const uint16_t *widths,
                    uint8_t n, barcode_result_t *r);

const char *barcode_action_name(uint8_t action);
const char *barcode_reject_name(uint8_t reject);

/* Collects bar_mbf records until END or ABORT. */
typedef struct {
    uint16_t w[B3_BAR_MAX_WIDTHS];
    uint8_t  n;
    bool     overflow;      /* more than [W_max] widths arrived */
} barcode_asm_t;

void barcode_asm_reset(barcode_asm_t *a);
/* Returns true when a barcode is complete (r filled: accepted or rejected). */
bool barcode_asm_push(barcode_asm_t *a, bar_rec_t rec,
                      const barcode_table_t *t, barcode_result_t *r);

#endif /* BARCODE_DECODE_H */