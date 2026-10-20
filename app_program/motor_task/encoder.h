/*
 *  encoder.h
 *  Both wheel encoders: quadrature, x4 decoding, GPIO edge interrupts.
 *
 *  Counts are published as one packed 32-bit word (left in bits 15:0,
 *  right in bits 31:16), written only by the GPIO interrupt with a single
 *  store.  Any task on either core reads both counts with one load, so
 *  there is no lock and no torn pair.  Use motion_unpack_left/right() and
 *  motion_count_delta() from motion_lib.h.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Set GP0..GP3 to inputs with pull-ups and attach to the GPIO dispatcher
   (gpio_irq_init() must already have run on this core).
   On a UART-console build GP0/GP1 carry the console, so the left encoder
   is left off and only the right one runs (encoder_left_enabled() says
   which).  The approved pin map uses the USB console for this reason. */
ER encoder_init(void);

uint32_t encoder_read_packed(void);    /* one load, both counts       */
uint32_t encoder_invalid_packed(void); /* missed-edge counts, same packing */
bool     encoder_left_enabled(void);

#endif /* ENCODER_H */
