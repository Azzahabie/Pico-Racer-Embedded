/*
 * bc_analog_partial.c - the partial-read decoder of the analog barcode
 * path.  Builds only with
 * BARCODE_SENSOR_ANALOG 1, so the digital car image is unchanged.
 */

#include "../common/line_config.h"

#if BARCODE_SENSOR_ANALOG

#include <stddef.h>

#include "bc_analog_partial.h"

#define T		(29u)		/* printed elements of *X*       */
#define INF		(0x3FFFFFFFu)
#define ERR_CAP		(1000000u)	/* one step 10 narrow off or more */
#define ROWS		(4u)		/* a step moves on 1 or 3 rows    */
#define KIND_MAX	((BC_PART_N_MAX + 1u) / 2u)

_Static_assert((BC_PART_CLIP_MAX % 2u) == 0u, "clips go by bar + space");
_Static_assert((BC_PART_CLIP_DP % 2u) == 0u, "clips go by bar + space");
_Static_assert(BC_PART_CLIP_DP <= 14u, "four bits");
_Static_assert(BC_PART_CLIP_MAX <= BC_PART_CLIP_DP, "the search covers what is accepted");

/* A path's counts, one word per cell. */
#define F_MB		(0u)		/* missed bars, 4 bits            */
#define F_MS		(4u)		/* missed spaces                  */
#define F_SP		(8u)		/* specks                         */
#define F_SA		(12u)		/* edits in the first '*', 2 bits */
#define F_SB		(14u)		/* edits in the last '*'          */
#define F_C0		(16u)		/* clipped before the first, 4    */
#define F_C1		(20u)		/* clipped after the last         */
#define F_ST		(24u)		/* worst step / 100, 8 bits       */
#define FIELD(x, s, m)	(((x) >> (s)) & (m))

typedef struct
{
	uint32_t const *w;
	uint16_t n;
	uint32_t lv[4];			/* bar n, bar W, space n, space W  */
	uint32_t inv;			/* 100 x 65536 / narrow           */
	uint32_t dmax;			/* largest difference inv scales  */
	uint32_t c1[BC_PART_N_MAX][2];	/* 1 : 1, narrow / wide printed   */
	uint32_t cs[BC_PART_N_MAX][2];	/* split at i, INF if no speck    */
	uint32_t ws[BC_PART_N_MAX];	/* split at i: the three, or 0    */
} ctx_t;

static ctx_t	s_c;
static char	s_cand[27];		/* GAMBLE candidates, "" = all   */
static uint32_t	s_cells;		/* since bc_partial_cells(true) */
static uint32_t	s_left;			/* this read's cells still allowed */

uint32_t bc_partial_cells(bool b_reset)
{
	uint32_t const c = s_cells;

	if (b_reset) {
		s_cells = 0u;
	}
	return c;
}

void bc_partial_set_candidates(const char *p_letters)
{
	uint16_t i = 0u;

	if (p_letters != NULL) {
		while ((i < 26u) && (p_letters[i] >= 'A') && (p_letters[i] <= 'Z')) {
			s_cand[i] = p_letters[i];
			i++;
		}
	}
	s_cand[i] = '\0';
}

void bc_partial_clear(bc_part_t *p_res)
{
	p_res->reason = (uint8_t)BC_PART_NOT_RUN;
	p_res->ch = '\0';
	p_res->b_reversed = false;
	p_res->ch2 = '\0';
	p_res->missed_bars = 0u;
	p_res->missed_spaces = 0u;
	p_res->specks = 0u;
	p_res->clip_front = 0u;
	p_res->clip_back = 0u;
	p_res->star_front = 0u;
	p_res->star_back = 0u;
	p_res->elements = 0u;
	p_res->step_max = 0u;
	p_res->cost = INF;
	p_res->cost2 = INF;
	p_res->width = INF;
}

/* Saturating add to a counter field. */
static uint32_t field_add(uint32_t f, uint32_t shift, uint32_t mask, uint32_t v)
{
	uint32_t x = FIELD(f, shift, mask) + v;

	x = (x > mask) ? mask : x;
	return (f & ~(mask << shift)) | (x << shift);
}

/* The worst step so far. */
static uint32_t with_step(uint32_t f, uint32_t step)
{
	uint32_t s = step / 100u;

	s = (s > 255u) ? 255u : s;
	return (s > (f >> F_ST)) ? ((f & 0x00FFFFFFu) | (s << F_ST)) : f;
}

/* An edit covering printed j_lo..j_hi: its count, and the '*' it is in. */
static uint32_t with_edit(uint32_t f, uint32_t shift, uint32_t v, uint16_t j_lo, uint16_t j_hi)
{
	f = field_add(f, shift, 0xFu, v);
	f = (j_lo <= 8u) ? field_add(f, F_SA, 3u, v) : f;
	return (j_hi >= 20u) ? field_add(f, F_SB, 3u, v) : f;
}

/* ---- this read's levels ----------------------------------------------- */

static uint32_t median_sorted(uint32_t const *s, uint16_t m)
{
	return ((m % 2u) != 0u) ? s[m / 2u] : ((s[(m / 2u) - 1u] + s[m / 2u]) / 2u);
}

/* Narrow and wide widths of one kind (bars from 0, spaces from 1): the
   kind's widths sorted and split in two where the classes are tightest
   (largest between-class variance), then the median of each side.
   Merged elements fall on the wide side or beyond, specks on the narrow
   side, so the medians hold.  False if there are fewer than 2. */
static bool kind_levels(uint32_t const *p_w, uint16_t n, uint16_t first, uint32_t *p_nar,
			uint32_t *p_wid)
{
	uint32_t s[KIND_MAX];
	uint32_t total = 0u;
	uint32_t left = 0u;
	uint64_t best = 0u;
	uint16_t split = 0u;
	uint16_t m = 0u;
	uint16_t i;
	uint16_t j;

	for (i = first; i < n; i = (uint16_t)(i + 2u)) {
		uint32_t const v = p_w[i] / 100u;	/* 0.1 mm, sums fit */

		for (j = m; (j > 0u) && (s[j - 1u] > v); j--) {
			s[j] = s[j - 1u];
		}
		s[j] = v;
		m++;
		total += v;
	}
	if (m < 2u) {
		return false;
	}
	/* between-class variance ~ (S0 n1 - S1 n0)^2 / (n0 n1) */
	for (i = 1u; i < m; i++) {
		uint32_t const n0 = i;
		uint32_t const n1 = (uint32_t)m - i;
		uint64_t       a;
		uint64_t       b;
		uint64_t       d;
		uint64_t       v;

		left += s[i - 1u];
		a = (uint64_t)left * n1;
		b = (uint64_t)(total - left) * n0;
		d = (a > b) ? (a - b) : (b - a);
		v = (d * d) / ((uint64_t)n0 * n1);
		if (v > best) {
			best = v;
			split = i;
		}
	}
	if (split == 0u) {
		return false;			/* all alike */
	}
	*p_nar = median_sorted(s, split) * 100u;
	*p_wid = median_sorted(&s[split], (uint16_t)(m - split)) * 100u;
	return true;
}

/* The blur moves every edge alike: bars read b thinner and spaces b
   wider whatever their width.  So the narrow pair gives the narrow width
   n and 2b, and the wide bars and wide spaces each give the wide width W.
   Merges only widen, so if the two disagree by more than half a narrow
   (both wide spaces of a read can be merges, a thin bar lost beside
   each), the wider one is taken from the other.  Then the printed
   wide / narrow must be within BC_PART_R_MIN_X10..BC_PART_R_MAX_X10
   (Code 39 prints 2 to 3), else the levels are not a code's. */
static bool levels_consistent(uint32_t *lv)
{
	int32_t const n2 = (int32_t)lv[0] + (int32_t)lv[2];		/* 2 n */
	int32_t const b2 = (int32_t)lv[2] - (int32_t)lv[0];		/* 2 b */
	int32_t const wb2 = (2 * (int32_t)lv[1]) + b2;			/* 2 W, bars   */
	int32_t const ws2 = (2 * (int32_t)lv[3]) - b2;			/* 2 W, spaces */
	int32_t       w2;

	if ((n2 < 200) || (wb2 <= 0) || (ws2 <= 0)) {
		return false;
	}
	if ((ws2 - wb2) > (n2 / 2)) {
		lv[3] = (uint32_t)((wb2 + b2) / 2);
	} else if ((wb2 - ws2) > (n2 / 2)) {
		lv[1] = (uint32_t)((ws2 - b2) / 2);
	} else {
		/* they agree */
	}
	if ((lv[1] <= lv[0]) || (lv[3] <= lv[2])) {
		return false;
	}
	w2 = (int32_t)lv[1] + (int32_t)lv[3];
	return ((w2 * 10) >= (n2 * (int32_t)BC_PART_R_MIN_X10)) && ((w2 * 10) <= (n2 * (int32_t)BC_PART_R_MAX_X10));
}

/* ---- the line-up ------------------------------------------------------ */

/* Width error squared, hundredths of a narrow.  An end element shorter
   than expected may be cut short: it costs BC_PART_CLIP_W_X16 / 16. */
static uint32_t err_cost(uint32_t meas, uint32_t expect, bool b_end)
{
	uint32_t const d = (meas >= expect) ? (meas - expect) : (expect - meas);
	uint32_t       e;

	if (d > s_c.dmax) {
		return ERR_CAP;
	}
	e = (d * s_c.inv) >> 16;
	if (e >= 1000u) {
		return ERR_CAP;
	}
	e = e * e;
	if (b_end && (meas < expect)) {
		e = (e * BC_PART_CLIP_W_X16) >> 4;
	}
	return e;
}

typedef struct
{
	uint32_t dp[ROWS][T + 1u];
	uint32_t fl[ROWS][T + 1u];
	uint32_t rmin[ROWS];
	uint32_t bound;
} grid_t;

static grid_t s_g;

/* The least the line-up of the last a measured and b printed elements
   can cost: two more measured than printed need a speck, two more
   printed a missed element, but up to BC_PART_CLIP_DP printed can be
   clipped at the end. */
static uint32_t rest_of(uint32_t a, uint32_t b)
{
	if (a > b) {
		return ((a - b) / 2u) * BC_PART_P_SPLIT;
	}
	return ((b - a) > BC_PART_CLIP_DP) ? (((b - a) - BC_PART_CLIP_DP) / 2u) * BC_PART_P_MISS : 0u;
}

/* If cost c improves cell (i, j) and is under the bound, it is stored
   and the cell's counts slot is returned for the caller to fill; else
   NULL (most steps: the counts are only worked out for the ones kept). */
static uint32_t *improve(uint16_t i, uint16_t j, uint32_t c)
{
	uint32_t const r = i % ROWS;

	if ((c >= s_g.dp[r][j]) || (c >= s_g.bound)) {
		return NULL;
	}
	s_g.dp[r][j] = c;
	s_g.rmin[r] = (c < s_g.rmin[r]) ? c : s_g.rmin[r];
	return &s_g.fl[r][j];
}

/* Cost of lining the read up with one template (wide flags and expected
   widths in reading order); *p_f gets the path's counts.  Gives up (INF)
   once every open path costs bound or more: costs only grow along a
   path, so such a template can change neither the best nor (b). */
static uint32_t fit(bool const *wide, uint32_t const *e, uint32_t bound, uint32_t *p_f)
{
	uint16_t const n = s_c.n;
	uint32_t       best = INF;
	uint16_t       i;
	uint16_t       j;
	uint16_t       r;

	for (r = 0u; r < ROWS; r++) {
		for (j = 0u; j <= T; j++) {
			s_g.dp[r][j] = INF;
		}
		s_g.rmin[r] = INF;
	}
	for (j = 0u; j <= BC_PART_CLIP_DP; j = (uint16_t)(j + 2u)) {
		s_g.dp[0][j] = (j / 2u) * BC_PART_P_CLIP;
		s_g.fl[0][j] = (uint32_t)j << F_C0;
	}
	s_g.rmin[0] = 0u;
	s_g.bound = bound;
	for (i = 0u; i < n; i++) {
		bool const     b_end = (i == 0u) || ((i + 1u) == n);
		uint32_t const kmiss = ((i % 2u) == 0u) ? F_MS : F_MB;	/* a bar merges spaces */
		uint32_t       open = s_g.rmin[i % ROWS];

		/* every open path has a cell in row i, i + 1 or i + 2 */
		open = (s_g.rmin[(i + 1u) % ROWS] < open) ? s_g.rmin[(i + 1u) % ROWS] : open;
		open = (((i + 2u) <= n) && (s_g.rmin[(i + 2u) % ROWS] < open)) ? s_g.rmin[(i + 2u) % ROWS] : open;
		if (open >= bound) {
			return INF;
		}
		r = (uint16_t)(i % ROWS);
		for (j = (uint16_t)(i & 1u); j < T; j = (uint16_t)(j + 2u)) {
			uint32_t const c = s_g.dp[r][j];
			uint32_t const f = s_g.fl[r][j];
			uint32_t      *p;
			uint32_t       step;
			uint32_t       sum;
			uint16_t       m;

			if ((c + rest_of((uint32_t)n - i, T - (uint32_t)j)) >= bound) {
				continue;		/* cannot finish under the bound */
			}
			if (s_left == 0u) {
				return INF;		/* BC_PART_CELLS_MAX: refused */
			}
			s_left--;
			s_cells++;
			/* 1 : 1 */
			step = s_c.c1[i][wide[j] ? 1u : 0u];
			p = improve((uint16_t)(i + 1u), (uint16_t)(j + 1u), c + step);
			if (p != NULL) {
				*p = with_step(f, step);
			}
			/* thin elements lost: 3, 5 or 7 printed in one */
			sum = e[j];
			for (m = 1u; (m <= BC_PART_MERGE_MAX) && ((j + (2u * m)) < T) && !wide[j + (2u * m) - 1u] &&
				     ((c + (m * BC_PART_P_MISS)) < bound); m++) {
				uint16_t const jh = (uint16_t)(j + (2u * m));

				sum += e[jh - 1u] + e[jh];
				step = err_cost(s_c.w[i], sum, b_end);
				p = improve((uint16_t)(i + 1u), (uint16_t)(jh + 1u), c + (m * BC_PART_P_MISS) + step);
				if (p != NULL) {
					*p = with_step(with_edit(f, kmiss, m, j, jh), step);
				}
			}
			if (((i + 3u) <= n) && (s_c.ws[i] != 0u)) {
				/* a speck: three measured for one printed */
				step = s_c.cs[i][wide[j] ? 1u : 0u];
				p = improve((uint16_t)(i + 3u), (uint16_t)(j + 1u), c + BC_PART_P_SPLIT + step);
				if (p != NULL) {
					*p = with_step(with_edit(f, F_SP, 1u, j, j), step);
				}
				/* in an element that also merged */
				sum = e[j];
				for (m = 1u; (m <= BC_PART_MERGE_MAX) && ((j + (2u * m)) < T) && !wide[j + (2u * m) - 1u] &&
					     ((c + BC_PART_P_SPLIT + (m * BC_PART_P_MISS)) < bound); m++) {
					uint16_t const jh = (uint16_t)(j + (2u * m));

					sum += e[jh - 1u] + e[jh];
					step = err_cost(s_c.ws[i], sum, false);
					p = improve((uint16_t)(i + 3u), (uint16_t)(jh + 1u),
						    c + BC_PART_P_SPLIT + (m * BC_PART_P_MISS) + step);
					if (p != NULL) {
						*p = with_step(with_edit(with_edit(f, F_SP, 1u, j, j), kmiss, m, j, jh), step);
					}
				}
			}
		}
		for (j = 0u; j <= T; j++) {
			s_g.dp[r][j] = INF;		/* this row becomes row i + 4 */
		}
		s_g.rmin[r] = INF;
	}
	r = (uint16_t)(n % ROWS);
	for (j = 0u; j <= BC_PART_CLIP_DP; j = (uint16_t)(j + 2u)) {
		uint32_t const c = s_g.dp[r][T - j];
		uint32_t const f = s_g.fl[r][T - j];

		if ((c < INF) && ((FIELD(f, F_C0, 0xFu) + j) <= BC_PART_CLIP_DP_TOTAL) &&
		    ((c + ((j / 2u) * BC_PART_P_CLIP)) < best)) {
			best = c + ((j / 2u) * BC_PART_P_CLIP);
			*p_f = f | ((uint32_t)j << F_C1);
		}
	}
	return best;
}

/* The runner-up only matters below this (b). */
static uint32_t margin_of(uint32_t c)
{
	uint64_t const a = ((uint64_t)c * BC_PART_MARGIN_X10) / 10u;
	uint64_t const b = (uint64_t)c + BC_PART_MARGIN_MIN;
	uint64_t const m = (a > b) ? a : b;

	return (m > INF) ? INF : (uint32_t)m;
}

static uint32_t penalty_of(uint32_t f)
{
	return ((FIELD(f, F_MB, 0xFu) + FIELD(f, F_MS, 0xFu)) * BC_PART_P_MISS) +
	       (FIELD(f, F_SP, 0xFu) * BC_PART_P_SPLIT) +
	       (((FIELD(f, F_C0, 0xFu) + FIELD(f, F_C1, 0xFu)) / 2u) * BC_PART_P_CLIP);
}

static void prepare(uint32_t const *p_w, uint16_t n, uint32_t nar)
{
	uint16_t i;

	s_c.w = p_w;
	s_c.n = n;
	s_c.inv = (100u << 16) / nar;
	s_c.dmax = 0xFFFFFFFFu / s_c.inv;
	for (i = 0u; i < n; i++) {
		uint32_t const *lv = &s_c.lv[((i % 2u) != 0u) ? 2u : 0u];
		bool const      b_end = (i == 0u) || ((i + 1u) == n);
		uint8_t         k;

		/* a speck: the middle one under BC_PART_SPECK_PCT % of a narrow
		   one of its kind */
		s_c.ws[i] = 0u;
		if (((i + 2u) < n) &&
		    (((uint64_t)p_w[i + 1u] * 100u) <
		     ((uint64_t)s_c.lv[((i % 2u) != 0u) ? 0u : 2u] * BC_PART_SPECK_PCT))) {
			s_c.ws[i] = p_w[i] + p_w[i + 1u] + p_w[i + 2u];
		}
		for (k = 0u; k < 2u; k++) {
			s_c.c1[i][k] = err_cost(p_w[i], lv[k], b_end);
			s_c.cs[i][k] = (s_c.ws[i] != 0u) ? err_cost(s_c.ws[i], lv[k], false) : INF;
		}
	}
}

code39_result_t bc_partial_decode(uint32_t const *p_w, uint16_t n, bc_part_t *p_res)
{
	uint16_t const star = code39_pattern('*');
	bool           wd[T];
	uint32_t       e[T];
	uint32_t       c1 = INF;		/* best, its letter and path */
	uint32_t       c2 = INF;		/* best of any other letter  */
	uint32_t       f1 = 0u;
	uint32_t       bound_max;
	uint32_t       nar;
	uint32_t       edits;
	char           letter;
	uint16_t       k;

	bc_partial_clear(p_res);
	p_res->elements = n;
	if ((n < ((BC_PART_GAMBLE != 0u) ? BC_PART_GAMBLE_N_MIN : BC_PART_N_MIN)) ||
	    (n > BC_PART_N_MAX) || ((n % 2u) == 0u)) {
		p_res->reason = (uint8_t)BC_PART_COUNT;
		return CODE39_ERR_COUNT;
	}
	if (!kind_levels(p_w, n, 0u, &s_c.lv[0], &s_c.lv[1]) ||
	    !kind_levels(p_w, n, 1u, &s_c.lv[2], &s_c.lv[3]) || !levels_consistent(s_c.lv)) {
		p_res->reason = (uint8_t)BC_PART_LEVELS;
		return CODE39_ERR_RATIO;
	}
	nar = (s_c.lv[0] + s_c.lv[2]) / 2u;
	prepare(p_w, n, nar);
	s_left = BC_PART_CELLS_MAX;
	/* the largest best that (a) and (d) can accept, with its penalties */
	bound_max = margin_of((BC_PART_ELEM_MAX * n) +
			      (BC_PART_K * ((BC_PART_P_MISS > BC_PART_P_SPLIT) ? BC_PART_P_MISS : BC_PART_P_SPLIT)) +
			      ((BC_PART_CLIP_MAX / 2u) * BC_PART_P_CLIP));
	{
	static const char all[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	const char *list = ((BC_PART_GAMBLE != 0u) && (s_cand[0] != '\0')) ? s_cand : all;
	uint16_t li;

	if (BC_PART_GAMBLE != 0u) {
		bound_max = INF;	/* GAMBLE: the true lowest, no rule bounds */
	}
	for (li = 0u; list[li] != '\0'; li++) {
		uint16_t const pat = code39_pattern(list[li]);

		letter = list[li];
		uint8_t        d;

		for (k = 0u; k < T; k++) {
			uint16_t const ci  = (uint16_t)(k / 10u);
			uint16_t const pos = (uint16_t)(k % 10u);
			uint16_t const p   = (ci == 1u) ? pat : star;

			wd[k] = (pos < 9u) && (((p >> (8u - pos)) & 1u) != 0u);
		}
		for (d = 0u; d < 2u; d++) {
			uint32_t bound = (c2 < bound_max) ? c2 : bound_max;
			uint32_t f = 0u;
			uint32_t cost;

			if (d == 1u) {
				for (k = 0u; k < (T / 2u); k++) {
					bool const x = wd[k];

					wd[k] = wd[T - 1u - k];
					wd[T - 1u - k] = x;
				}
			}
			for (k = 0u; k < T; k++) {
				e[k] = s_c.lv[(((k % 2u) != 0u) ? 2u : 0u) + (wd[k] ? 1u : 0u)];
			}
			if (BC_PART_GAMBLE != 0u) {
				bound = (c1 < bound) ? c1 : bound;	/* only a strictly lower cost wins */
			} else {
				bound = (margin_of(c1) < bound) ? margin_of(c1) : bound;
			}
			cost = fit(wd, e, bound, &f);
			if (cost < c1) {
				if (letter != p_res->ch) {
					c2 = c1;
					p_res->ch2 = p_res->ch;
				}
				c1 = cost;
				f1 = f;
				p_res->ch = letter;
				p_res->b_reversed = (d == 1u);
			} else if ((cost < c2) && (letter != p_res->ch)) {
				c2 = cost;
				p_res->ch2 = letter;
			} else {
				/* changes nothing */
			}
		}
	}
	}
	if ((BC_PART_GAMBLE != 0u) && (p_res->ch != '\0') && (c1 < INF)) {
		/* GAMBLE: the best candidate, whatever its cost (a search cut by
		   the cell limit still gives the best found so far) */
		p_res->cost = c1;
		p_res->cost2 = (c2 < INF) ? c2 : 0u;
		p_res->width = c1 - penalty_of(f1);
		p_res->missed_bars = (uint8_t)FIELD(f1, F_MB, 0xFu);
		p_res->missed_spaces = (uint8_t)FIELD(f1, F_MS, 0xFu);
		p_res->specks = (uint8_t)FIELD(f1, F_SP, 0xFu);
		p_res->clip_front = (uint8_t)FIELD(f1, F_C0, 0xFu);
		p_res->clip_back = (uint8_t)FIELD(f1, F_C1, 0xFu);
		p_res->reason = (uint8_t)BC_PART_OK;
		return CODE39_OK;
	}
	if (s_left == 0u) {
		p_res->reason = (uint8_t)BC_PART_BUSY;	/* not searched to the end */
		return CODE39_ERR_PATTERN;
	}
	if (p_res->ch == '\0') {
		p_res->reason = (uint8_t)BC_PART_COST;
		return CODE39_ERR_PATTERN;
	}
	p_res->cost = c1;
	p_res->cost2 = (c2 < INF) ? c2 : margin_of(c1);
	p_res->width = c1 - penalty_of(f1);
	p_res->missed_bars = (uint8_t)FIELD(f1, F_MB, 0xFu);
	p_res->missed_spaces = (uint8_t)FIELD(f1, F_MS, 0xFu);
	p_res->specks = (uint8_t)FIELD(f1, F_SP, 0xFu);
	p_res->clip_front = (uint8_t)FIELD(f1, F_C0, 0xFu);
	p_res->clip_back = (uint8_t)FIELD(f1, F_C1, 0xFu);
	p_res->star_front = (uint8_t)FIELD(f1, F_SA, 3u);
	p_res->star_back = (uint8_t)FIELD(f1, F_SB, 3u);
	p_res->step_max = (uint16_t)(FIELD(f1, F_ST, 0xFFu) * 100u);
	edits = (uint32_t)p_res->missed_bars + p_res->missed_spaces + p_res->specks;
	if ((p_res->width > (BC_PART_ELEM_MAX * n)) || (p_res->step_max > BC_PART_STEP_MAX)) {
		p_res->reason = (uint8_t)BC_PART_COST;
	} else if (edits > BC_PART_K) {
		p_res->reason = (uint8_t)BC_PART_EDITS;
	} else if ((p_res->clip_front > BC_PART_CLIP_MAX) || (p_res->clip_back > BC_PART_CLIP_MAX) ||
		   !(((p_res->clip_front == 0u) && (p_res->star_front <= BC_PART_STAR_EDITS)) ||
		     ((p_res->clip_back == 0u) && (p_res->star_back <= BC_PART_STAR_EDITS)))) {
		p_res->reason = (uint8_t)BC_PART_STAR;
	} else if (c2 < margin_of(c1)) {
		p_res->reason = (uint8_t)BC_PART_MARGIN;
	} else {
		p_res->reason = (uint8_t)BC_PART_OK;
		return CODE39_OK;
	}
	return CODE39_ERR_PATTERN;
}

#else
typedef int bc_analog_partial_off_t;	/* digital image: nothing built here */
#endif /* BARCODE_SENSOR_ANALOG */
