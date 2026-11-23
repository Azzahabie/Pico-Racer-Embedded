#ifndef ROBOCAR_BC_ANALOG_PARTIAL_H
#define ROBOCAR_BC_ANALOG_PARTIAL_H

/*
 * bc_analog_partial.h - the partial-read decoder of the analog barcode
 * path (BARCODE_SENSOR_ANALOG 1).  Ours, not the other team's.
 *
 * The last resort, run by bc_finish() (core 0) only after every exact
 * decoder, the repairs and the other team's best match have failed.  A read that
 * lost a few thin elements is lined up (dynamic programming) with the 29
 * printed elements of *X* for every X in A-Z, read forwards and
 * backwards.  The steps of a line-up:
 *   1 measured : 1 printed;
 *   1 measured : 3, 5 or 7 printed, missed thin elements: a narrow bar
 *     lost between two spaces gives one long space, a narrow space lost
 *     between two bars one long bar (up to BC_PART_MERGE_MAX in a row);
 *   3 measured : 1 printed, a speck splits one element (the middle one
 *     thinner than BC_PART_SPECK_PCT % of a narrow one of its kind), or
 *     3 : 3, a speck in an element that also merged;
 *   printed elements not seen at either end (clipped), an even number
 *     (a read starts and ends on a bar).
 * The expected width of each printed element is this read's own narrow or
 * wide bar or space (bars and spaces apart, as the sensor's blur makes
 * bars thinner and spaces wider); a merged element is expected as the
 * sum of what it merged.  Each step costs its width error squared, in
 * hundredths of a narrow element (scale free), plus a penalty per missed
 * element, speck and clipped pair.
 *
 * A letter is returned only if all of these hold:
 *   (a) its width error is at most BC_PART_ELEM_MAX per element, and no
 *       single step costs more than BC_PART_STEP_MAX;
 *   (b) every other letter costs at least BC_PART_MARGIN_X10 / 10 x as
 *       much, and at least BC_PART_MARGIN_MIN more;
 *   (c) one of the two '*' is seen whole (nothing clipped from it) with at
 *       most BC_PART_STAR_EDITS missed elements or specks in it, and at
 *       most BC_PART_CLIP_MAX printed elements are clipped at the other
 *       end;
 *   (d) at most BC_PART_K missed elements and specks in all.
 * The other letters are lined up more generously (any number of missed
 * elements, merged and split ones, up to BC_PART_CLIP_DP clipped at each
 * end), so a read with more damage than the rules accept still finds its
 * true letter as the runner-up and is refused by (b).
 *
 * CPU: line-ups give up once they cannot beat the best so far or the
 * margin (b); at most BC_PART_CELLS_MAX cells per read, then refused
 * (BC_PART_BUSY).
 *
 * Pure C (host-tested in tests/line).
 */

#include <stdbool.h>
#include <stdint.h>

#include "bc_analog_code39.h"

/* ---- settings (-D) ---------------------------------------------------- */
/* Costs are width errors squared in hundredths of a narrow element:
   2500 = an error of half a narrow.  The values are chosen from the
   model in tests/line/test_bc_analog_partial.c (sim), see
   docs/BUDDY3_LINE_BARCODE.md. */
#ifndef BC_PART_K
#define BC_PART_K		(2u)	/* (d) missed elements + specks   */
#endif
#ifndef BC_PART_P_MISS
#define BC_PART_P_MISS		(2500u)	/* per missed thin element         */
#endif
#ifndef BC_PART_P_SPLIT
#define BC_PART_P_SPLIT		(2500u)	/* per speck                       */
#endif
#ifndef BC_PART_P_CLIP
#define BC_PART_P_CLIP		(1000u)	/* per clipped bar + space         */
#endif
#ifndef BC_PART_CLIP_MAX
#define BC_PART_CLIP_MAX	(0u)	/* (c) clipped at either end       */
#endif
#ifndef BC_PART_CLIP_DP
#define BC_PART_CLIP_DP		(14u)	/* the search, at each end         */
#endif
#ifndef BC_PART_CLIP_DP_TOTAL
#define BC_PART_CLIP_DP_TOTAL	(18u)	/* the search, both ends           */
#endif
#ifndef BC_PART_MERGE_MAX
#define BC_PART_MERGE_MAX	(3u)	/* missed in a row in one element  */
#endif
#ifndef BC_PART_CLIP_W_X16
#define BC_PART_CLIP_W_X16	(4u)	/* an end element cut short: 1/4   */
#endif
#ifndef BC_PART_ELEM_MAX
#define BC_PART_ELEM_MAX	(900u)	/* (a) per measured element        */
#endif
#ifndef BC_PART_STEP_MAX
#define BC_PART_STEP_MAX	(16000u) /* (a) one step: 1.26 narrow off  */
#endif
#ifndef BC_PART_MARGIN_X10
#define BC_PART_MARGIN_X10	(30u)	/* (b) runner-up / best            */
#endif
#ifndef BC_PART_MARGIN_MIN
#define BC_PART_MARGIN_MIN	(10000u) /* (b) runner-up - best           */
#endif
#ifndef BC_PART_STAR_EDITS
#define BC_PART_STAR_EDITS	(1u)	/* (c) in the '*' seen whole       */
#endif
#ifndef BC_PART_SPECK_PCT
#define BC_PART_SPECK_PCT	(75u)	/* a speck: under 3/4 of a narrow  */
#endif
#ifndef BC_PART_CELLS_MAX
#define BC_PART_CELLS_MAX	(8000u)	/* line-up cells per read (CPU)    */
#endif
#ifndef BC_PART_R_MIN_X10
#define BC_PART_R_MIN_X10	(18u)	/* printed wide / narrow, at least */
#endif
#ifndef BC_PART_R_MAX_X10
#define BC_PART_R_MAX_X10	(34u)	/* and at most (Code 39: 2 to 3)   */
#endif

/* GAMBLE (ours, 21 Nov, demo day: only 3-4 letters have an action).  1:
   the line-up runs only over the candidate letters (bc_partial_set_
   candidates(), from the barcode map: every letter whose action is not
   straight on, A first), and the lowest cost simply wins: rules (a)-(d)
   are not applied.  A tie goes to the first candidate.  Reads of
   BC_PART_GAMBLE_N_MIN elements and up are tried (a barcode-like read:
   narrow and wide must still separate, BC_PART_LEVELS). */
#ifndef BC_PART_GAMBLE
#define BC_PART_GAMBLE		(0u)
#endif
#ifndef BC_PART_GAMBLE_N_MIN
#define BC_PART_GAMBLE_N_MIN	(11u)
#endif

/* Element counts the decoder takes (odd only; others BC_PART_COUNT). */
#define BC_PART_N_MAX		(35u)
/* fewer cannot meet (c) and (d) */
#define BC_PART_N_MIN		((((2u * BC_PART_K) + BC_PART_CLIP_MAX) < 20u) ? \
				 (29u - BC_PART_CLIP_MAX - (2u * BC_PART_K)) : 9u)

/* Why a read was refused (bc_part_t.reason). */
typedef enum
{
	BC_PART_OK      = 0,
	BC_PART_NOT_RUN = 1,	/* not tried: an earlier decoder decided   */
	BC_PART_COUNT   = 2,	/* too few or too many elements, or even   */
	BC_PART_LEVELS  = 3,	/* narrow and wide not separable           */
	BC_PART_COST    = 4,	/* (a) the best fit is too poor            */
	BC_PART_MARGIN  = 5,	/* (b) another letter fits nearly as well  */
	BC_PART_STAR    = 6,	/* (c) no '*' seen whole, or too clipped   */
	BC_PART_EDITS   = 7,	/* (d) more missed elements than allowed   */
	BC_PART_BUSY    = 8	/* the search reached BC_PART_CELLS_MAX    */
} bc_part_reason_t;

typedef struct
{
	uint8_t  reason;	/* bc_part_reason_t                        */
	char     ch;		/* best letter, also when refused; 0 none  */
	bool     b_reversed;
	char     ch2;		/* the next best other letter; 0 none      */
	uint8_t  missed_bars;	/* thin bars lost (merged spaces)          */
	uint8_t  missed_spaces;	/* thin spaces lost (merged bars)          */
	uint8_t  specks;	/* elements split by a speck               */
	uint8_t  clip_front;	/* printed elements before the first read  */
	uint8_t  clip_back;	/* printed elements after the last read    */
	uint8_t  star_front;	/* edits in the first '*' read             */
	uint8_t  star_back;	/* edits in the last '*' read              */
	uint16_t elements;	/* measured elements                       */
	uint16_t step_max;	/* the worst single step (to 100)          */
	uint32_t cost;		/* the best letter's cost                  */
	uint32_t cost2;		/* the next letter's, or the bound it lost */
	uint32_t width;		/* the best's cost without its penalties   */
} bc_part_t;

void bc_partial_clear(bc_part_t *p_res);

/* GAMBLE: the candidate letters, 'A'-'Z' in priority order (a tie goes to
   the first), NUL-terminated, at most 26; "" or NULL = every letter.
   Called by barcode_task (core 0, the same task that decodes) when the
   barcode map changes. */
void bc_partial_set_candidates(const char *p_letters);

/* The widths (um, bar first) of one reading.  CODE39_OK with
   p_res->ch, else CODE39_ERR_COUNT (count), CODE39_ERR_RATIO (levels) or
   CODE39_ERR_PATTERN (refused by a rule; p_res->reason says which). */
code39_result_t bc_partial_decode(uint32_t const *p_w, uint16_t n, bc_part_t *p_res);

/* Line-up cells evaluated since the last call with b_reset (host test:
   the CPU bound). */
uint32_t bc_partial_cells(bool b_reset);

#endif /* ROBOCAR_BC_ANALOG_PARTIAL_H */
