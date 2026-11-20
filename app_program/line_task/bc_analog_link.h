#ifndef ROBOCAR_BC_ANALOG_LINK_H
#define ROBOCAR_BC_ANALOG_LINK_H

/*
 * bc_analog_link.h - how the analog barcode path (BARCODE_SENSOR_ANALOG 1)
 * crosses from ir_sense_task (core 1) to barcode_task (core 0).
 *
 *   ir_sense_task --bar_mbf LETTER {4, read_id, letter | reversed << 8}--> barcode_task
 *       a read that ENDed at its stop character (BC_END_ON_STOP), already
 *       decoded on core 1: barcode_task looks the letter up and stores the
 *       command and the judged word, as for a GP7 read.
 *   ir_sense_task --bar_mbf ANALOG {5, read_id, BAR_ANALOG_CUT or 0}--> barcode_task
 *       The end of a code: the capture (line_bc_capture()) waits in
 *       BC_ST_DECODING; barcode_task runs bc_finish() on it (the re-reads
 *       and best match, up to milliseconds), stores the command and the
 *       judged word if it decoded, then the reply word, and only then
 *       prints.  ir_sense_task polls the reply word every 1 ms and gives
 *       the capture back to the states (bc_resume()).
 *   ir_sense_task --bar_mbf ABORT {3, read_id, BAR_ABORT_CANCELLED}--> barcode_task
 *       a mission change dropped a capture: barcode_rx.c judges it as for
 *       GP7 ("REJECTED: aborted (cancelled)").
 *
 * Single writers: the capture belongs to core 1 except in BC_ST_DECODING,
 * from the ANALOG message until core 1 sees the reply; the reply word is
 * written by barcode_task only (one aligned store, after a barrier that
 * ends its reads of the capture) and read by ir_sense_task only.
 */

#include <stdbool.h>
#include <stdint.h>

#include "bc_analog_capture.h"

#define BAR_MSG_LETTER	((uint8_t)4)
#define BAR_MSG_ANALOG	((uint8_t)5)
#define BAR_ANALOG_CUT	(1u)		/* ANALOG value: cut by a junction, judge only */

/* LETTER's 16-bit value. */
static inline uint16_t bc_link_letter_value(char letter, bool reversed)
{
	return (uint16_t)((uint16_t)(uint8_t)letter | (reversed ? 0x100u : 0u));
}

/*
 * The reply word: bit 8 set once any reply was stored, bits 0-7 the
 * read_id it answers, bits 9-10 bc_finish()'s result (BC_NONE, BC_VALID,
 * BC_REJECT), bits 16-23 the letter when BC_VALID.  0 before the first.
 */
#define BC_LINK_REPLY_SET	(0x100u)

static inline uint32_t bc_link_reply_pack(uint8_t read_id, bc_result_t result, char letter)
{
	return BC_LINK_REPLY_SET | (uint32_t)read_id |
	       (((uint32_t)result & 3u) << 9) | ((uint32_t)(uint8_t)letter << 16);
}

static inline bool bc_link_reply_is(uint32_t word, uint8_t read_id)
{
	return ((word & BC_LINK_REPLY_SET) != 0u) && ((word & 0xFFu) == read_id);
}

static inline bc_result_t bc_link_reply_result(uint32_t word)
{
	return (bc_result_t)((word >> 9) & 3u);
}

static inline char bc_link_reply_letter(uint32_t word)
{
	return (char)((word >> 16) & 0xFFu);
}

/* Orders this core's memory accesses before and after it (both cores). */
#if defined(__arm__)
#define BC_LINK_BARRIER()	__asm__ volatile ("dmb" ::: "memory")
#else
#define BC_LINK_BARRIER()	__atomic_thread_fence(__ATOMIC_SEQ_CST)
#endif

/* ir_sense_task's capture (line_task.c), for barcode_task while it is in
   BC_ST_DECODING. */
const bc_capture_t *line_bc_capture(void);

/* barcode_task's reply word (barcode_task.c), one aligned load. */
uint32_t barcode_analog_reply_word(void);

#endif
