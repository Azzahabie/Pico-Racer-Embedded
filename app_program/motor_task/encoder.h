/*
 *  encoder.h
 *  Both wheel encoders: quadrature, x4 decoding, GPIO edge interrupts.
 *
 *  Both wheel counts are 16-bit, wrapping; take changes with
 *  motion_count_delta() from motion_lib.h.  How the interrupt hands them
 *  to motion_task depends on MOTION_PACKED (motion_cfg.h):
 *    1  one 32-bit word (left bits 15:0, right bits 31:16), one store by
 *       the interrupt, one load by the reader: no lock, no torn pair.
 *    0  Week 6 form: one 32-bit counter per wheel, copied by the reader
 *       with interrupts briefly disabled.  Correct only because the
 *       interrupt and motion_task both run on core 1.
 *  encoder_read_counts() hides the difference from motion_task.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Set GP0..GP3 to inputs with pull-ups and attach them, one pin at a time,
   to Buddy 5's shared GPIO dispatcher (common/gpio_irq.h; gpio_irq_init()
   must already have run on this core, core 1).
   On a UART-console build GP0/GP1 carry the console, so the left encoder
   is left off and only the right one runs (encoder_left_enabled() says
   which).  The approved pin map uses the USB console for this reason. */
ER encoder_init(void);

/* Both counts, taken at the same moment.  Call from core 1 only. */
void     encoder_read_counts(uint16_t *p_left, uint16_t *p_right);
uint32_t encoder_invalid_packed(void); /* missed edges: left 15:0, right 31:16 */
bool     encoder_left_enabled(void);

/* Edges seen on each phase of one wheel: phase A in bits 15:0, phase B
   in bits 31:16, both wrapping (16-bit).  Take the change since the last
   read with (uint16_t)(now - prev) on each half.  The stall check uses
   them to tell a dead encoder wire (edges on one phase only) from a
   stopped wheel (no edges).  Written only by the interrupt, one store. */
uint32_t encoder_read_edges_left(void);
uint32_t encoder_read_edges_right(void);

/* Timing evidence: interrupts that reached the encoder callback (wrapping
   count), and the longest dispatcher-entry-to-callback-end time in us
   since the last take (a lower bound: it misses the exception entry).
   Call the take only from motion_task, on the encoder's core. */
uint32_t encoder_irq_count(void);
uint32_t encoder_take_irq_max_us(void);

#endif /* ENCODER_H */
