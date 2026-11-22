/* Host test for the partial-read decoder (bc_analog_partial.c) and
   its place in bc_finish(), after the decoders (pass 7):
     - the car reads of 20 Nov that were rejected (M): P1 (17:44, a 27
       that is *F* read backwards with one thin bar lost) must decode 'F';
       P2 (18:00, 15 elements) and P3 (17:30, 17) must give no letter;
     - every letter both ways with each single thin element lost (exact
       widths): never another letter;
     - the property test on the model in bc_partial_model.h (sim): every
       letter, both ways, three scales, 0 to BC_PART_K + 2 thin elements
       missed, clipped ends, specks, blur bias, noise, speed drift; reads
       distorted by a speed wobble; pure noise; reads cut to one symbol or
       a little more.  Decode rate and WRONG letters per decoder are
       printed; the partial decoder must give no wrong letter, the earlier
       decoders' wrong letters are printed (they were there before);
     - the CPU bound: cells per call and host time.
   The four car traces (fixtures/) are checked, through the whole path, by
   test_bc_analog_fixtures. */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "check.h"
#include "bc_partial_model.h"
#include "../../app_program/line_task/bc_analog_capture.h"

#ifndef PROP_PER
#define PROP_PER	(300u)		/* reads per letter, direction and class */
#endif
#ifndef PROP_SEED
#define PROP_SEED	(31337u)	/* not the seeds the settings were chosen on */
#endif

static bc_capture_t g_bc;

static uint16_t parse(const char *s, uint32_t *w)
{
	uint16_t n = 0u;

	while (*s != '\0') {
		if ((*s == 'B') || (*s == 'w')) {
			w[n++] = (uint32_t)lround(strtod(s + 1, NULL) * 1000.0);
		}
		s++;
	}
	return n;
}

/* bc_finish() on one reading: the capture holds these widths and no
   darkness profile (readings 1-5 empty), as the car decodes reading 0. */
static bc_result_t finish_one(const uint32_t *w, uint16_t n, bc_out_t *o)
{
	uint32_t sum = 0u;
	uint16_t k;

	bc_init(&g_bc);
	for (k = 0u; (k < n) && (k <= BC_MAX_ELEMENTS); k++) {
		g_bc.widths[k] = w[k];
		sum += w[k];
	}
	g_bc.n = n;
	g_bc.run_um = 40000u;
	g_bc.total_um = sum + g_bc.run_um;
	g_bc.state = (uint8_t)BC_ST_DECODING;
	return bc_finish(&g_bc, o);
}

/* ---- the car reads of 20 Nov ------------------------------------------ */

static const char k_p1[] =
	"B1.8 w10.9 B11.8 w11.7 B11.3 w34.8 B1.7 w9.6 B2.6 w8.0 B3.1 w20.5 B12.9 w11.7 B12.3 "
	"w10.5 B2.3 w9.4 B2.4 w9.0 B14.5 w8.5 B14.7 w9.1 B2.5 w21.3 B1.5";
static const char k_p2[] =
	"B3.4 w9.7 B1.9 w21.8 B11.1 w13.1 B11.5 w11.8 B1.3 w10.4 B1.4 w9.4 B13.0 w10.2 B11.9";
static const char k_p3[] =
	"B6.6 w5.0 B5.3 w2.5 B2.2 w33.7 B5.5 w4.5 B7.2 w5.4 B1.4 w1.1 B16.7 w2.1 B8.7 w10.9 B4.7";

static void test_car_reads(void)
{
	uint32_t    w[64];
	uint16_t    n = parse(k_p1, w);
	bc_part_t   pt;
	bc_out_t    o;
	bc_result_t r;

	/* P1: *F* backwards, read element 5 (w34.8) is a narrow space, a
	   lost thin bar and a wide space of the first '*' read */
	CHECK(n == 27u);
	CHECK(bc_partial_decode(w, n, &pt) == CODE39_OK);
	CHECK((pt.ch == 'F') && pt.b_reversed);
	CHECK((pt.missed_bars == 1u) && (pt.missed_spaces == 0u) && (pt.specks == 0u));
	CHECK((pt.clip_front == 0u) && (pt.clip_back == 0u));
	(void)printf("P1 (M, 17:44): '%c' %s, %u missed bar, cost %u, next '%c' %u (x%.1f)\n", pt.ch,
		     pt.b_reversed ? "rev" : "fwd", (unsigned)pt.missed_bars, (unsigned)pt.cost,
		     pt.ch2, (unsigned)pt.cost2, (double)pt.cost2 / (double)pt.cost);
	/* through bc_finish(): the decoders give nothing (the best match
	   never runs: trim_specks() cuts it to 23), the partial one 'F' */
	r = finish_one(w, n, &o);
	CHECK((r == BC_VALID) && (o.ch == 'F') && o.b_reversed && (o.pass == 7u) && (o.elements == 27u));
	CHECK(o.part.reason == (uint8_t)BC_PART_OK);

	/* P2 and P3: too short for any decoder; no letter */
	n = parse(k_p2, w);
	r = finish_one(w, n, &o);
	CHECK((n == 15u) && (r == BC_REJECT) && (o.part.reason == (uint8_t)BC_PART_COUNT));
	n = parse(k_p3, w);
	r = finish_one(w, n, &o);
	CHECK((n == 17u) && (r == BC_REJECT) && (o.part.reason == (uint8_t)BC_PART_COUNT));
}

/* ---- exact widths, one thin element lost ------------------------------- */

static uint16_t code_widths(char c, uint32_t nar, uint32_t r10, int rev, uint32_t *w)
{
	uint16_t k;

	for (k = 0u; k < 29u; k++) {
		uint16_t const kk = rev ? (uint16_t)(28u - k) : k;
		const char    *pp = pm_pat[((kk / 10u) == 1u) ? (c - 'A') : 26];
		bool const     wide = ((kk % 10u) < 9u) && (pp[kk % 10u] == '1');

		w[k] = wide ? (nar * r10) / 10u : nar;
	}
	return 29u;
}

static void test_one_lost(void)
{
	uint32_t w[32];
	uint32_t m[32];
	unsigned right = 0u;
	unsigned wrong = 0u;
	unsigned refused = 0u;
	unsigned clean = 0u;
	char     c;
	int      d;
	uint32_t r10;

	for (c = 'A'; c <= 'Z'; c++) {
		for (d = 0; d < 2; d++) {
			for (r10 = 22u; r10 <= 30u; r10 += 4u) {
				uint16_t  n = code_widths(c, 5100u, r10, d, w);
				uint16_t  k;
				bc_part_t pt;

				/* the whole read: the letter, nothing lost */
				if ((bc_partial_decode(w, n, &pt) == CODE39_OK) && (pt.ch == c) &&
				    (pt.missed_bars + pt.missed_spaces + pt.specks == 0u)) {
					clean++;
				}
				for (k = 1u; (k + 1u) < n; k++) {
					uint16_t j;
					uint16_t o = 0u;

					if (w[k] != 5100u) {
						continue;	/* only thin ones are lost */
					}
					for (j = 0u; j < n; j++) {
						if (j == (k - 1u)) {
							m[o++] = w[j] + w[j + 1u] + w[j + 2u];
							j = (uint16_t)(j + 2u);
						} else {
							m[o++] = w[j];
						}
					}
					if (bc_partial_decode(m, o, &pt) != CODE39_OK) {
						refused++;
					} else if (pt.ch == c) {
						right++;
					} else {
						wrong++;
					}
				}
			}
		}
	}
	(void)printf("one thin element lost (exact widths, wide 2.2 / 2.6 / 3.0 x): %u right, %u refused, "
		     "%u WRONG; whole reads %u of 156\n", right, refused, wrong, clean);
	CHECK(wrong == 0u);
	CHECK(clean == 156u);
	CHECK(right > refused);
}

/* ---- the property test -------------------------------------------------- */

enum { DEC_NONE = 0, DEC_EXACT, DEC_BEST, DEC_PART, DEC_N };

typedef struct {
	unsigned reads;
	unsigned right[DEC_N];
	unsigned wrong[DEC_N];
} tally_t;

static int decoder_of(bc_result_t r, const bc_out_t *o)
{
	if (r != BC_VALID) {
		return DEC_NONE;
	}
	if (o->pass == 7u) {
		return DEC_PART;
	}
	return ((o->pass == 4u) || (o->pass == 6u)) ? DEC_BEST : DEC_EXACT;
}

static void tally(tally_t *t, char truth, const uint32_t *w, uint16_t n)
{
	bc_out_t          o;
	bc_result_t const r = finish_one(w, n, &o);
	int const         dec = decoder_of(r, &o);

	t->reads++;
	if (dec == DEC_NONE) {
		return;
	}
	if (o.ch == truth) {
		t->right[dec]++;
	} else if ((o.ch >= 'A') && (o.ch <= 'Z')) {
		t->wrong[dec]++;		/* another letter: turns the car */
	} else {
		/* a digit or symbol: no table entry, rejected as unknown */
	}
}

static void print_tally(const char *name, const tally_t *t)
{
	double const all = (t->reads > 0u) ? (double)t->reads : 1.0;

	(void)printf("  %-34s %6u | right %6.2f %% (exact %5.2f, best %5.2f, partial %5.2f) | "
		     "WRONG exact %u, best %u, partial %u\n", name, t->reads,
		     100.0 * (t->right[DEC_EXACT] + t->right[DEC_BEST] + t->right[DEC_PART]) / all,
		     100.0 * t->right[DEC_EXACT] / all, 100.0 * t->right[DEC_BEST] / all,
		     100.0 * t->right[DEC_PART] / all, t->wrong[DEC_EXACT], t->wrong[DEC_BEST],
		     t->wrong[DEC_PART]);
}

static void test_property(void)
{
	static tally_t miss_t[BC_PART_K + 3u];
	tally_t        dist = { 0u, { 0u }, { 0u } };
	tally_t        noise = { 0u, { 0u }, { 0u } };
	tally_t        cut1 = { 0u, { 0u }, { 0u } };
	tally_t        cut2 = { 0u, { 0u }, { 0u } };
	pm_opt_t       opt;
	unsigned       total = 0u;
	unsigned       part_wrong = 0u;
	unsigned       k;
	int            miss;
	char           c;
	int            d;

	pm_rng = PROP_SEED;
	pm_opt_default(&opt);
	(void)printf("property test (sim, bc_partial_model.h; K %u, margin x%u.%u, seed %u):\n",
		     (unsigned)BC_PART_K, (unsigned)(BC_PART_MARGIN_X10 / 10u),
		     (unsigned)(BC_PART_MARGIN_X10 % 10u), (unsigned)PROP_SEED);
	for (miss = 0; miss <= (int)(BC_PART_K + 2u); miss++) {
		char name[48];

		opt.miss = miss;
		for (c = 'A'; c <= 'Z'; c++) {
			for (d = 0; d < 2; d++) {
				for (k = 0u; k < PROP_PER; k++) {
					uint32_t   w[64];
					pm_truth_t t;
					int const  n = pm_read(c, d, &opt, w, &t);

					tally(&miss_t[miss], c, w, (uint16_t)n);
				}
			}
		}
		(void)snprintf(name, sizeof(name), "%d thin element%s missed", miss, (miss == 1) ? "" : "s");
		print_tally(name, &miss_t[miss]);
		total += miss_t[miss].reads;
		part_wrong += miss_t[miss].wrong[DEC_PART];
	}
	/* distorted: speed wobble up to 40 %, drift up to 30 % */
	pm_opt_default(&opt);
	opt.wobble = 0.4;
	opt.drift = 0.3;
	for (k = 0u; k < (52u * PROP_PER); k++) {
		uint32_t   w[64];
		pm_truth_t t;
		char const l = (char)('A' + (pm_rnd() % 26u));
		int        n;

		opt.miss = (int)(pm_rnd() % 4u);
		n = pm_read(l, (int)(pm_rnd() & 1u), &opt, w, &t);
		tally(&dist, l, w, (uint16_t)n);
	}
	print_tally("distorted (wobble, drift)", &dist);
	/* pure noise: any letter is wrong */
	for (k = 0u; k < (52u * PROP_PER); k++) {
		uint32_t w[64];
		int const n = pm_noise(w, (int)(k % 3u));

		tally(&noise, '?', w, (uint16_t)n);
	}
	print_tally("pure noise (any letter is wrong)", &noise);
	/* cut to one symbol (9..13 elements) or a little more (15..21) */
	pm_opt_default(&opt);
	opt.clip_pct = 0;
	opt.speck_pct = 0;
	for (k = 0u; k < (52u * PROP_PER); k++) {
		uint32_t   w[64];
		pm_truth_t t;
		char const l = (char)('A' + (pm_rnd() % 26u));
		int        n;
		int        len;
		int        off;
		bool const b_one = ((k % 2u) == 0u);

		opt.miss = (int)(pm_rnd() % 3u);
		n = pm_read(l, (int)(pm_rnd() & 1u), &opt, w, &t);
		len = b_one ? (9 + (2 * (int)(pm_rnd() % 3u))) : (15 + (2 * (int)(pm_rnd() % 4u)));
		len = (len > n) ? n : len;
		off = 2 * (int)(pm_rnd() % (uint32_t)(((n - len) / 2) + 1));
		tally(b_one ? &cut1 : &cut2, l, &w[off], (uint16_t)len);
	}
	print_tally("cut to one symbol (9..13)", &cut1);
	print_tally("cut to 15..21 elements", &cut2);
	total += dist.reads + noise.reads + cut1.reads + cut2.reads;
	part_wrong += dist.wrong[DEC_PART] + noise.wrong[DEC_PART] + cut1.wrong[DEC_PART] +
		      cut2.wrong[DEC_PART];
	(void)printf("  %u reads: the partial decoder gave %u WRONG letters\n", total, part_wrong);

	/* the safety rule: never another letter from the partial decoder */
	CHECK(part_wrong == 0u);
	CHECK(total >= 100000u);
	/* and it is worth having: most single missed thin elements decode */
	CHECK((miss_t[1].right[DEC_PART] * 100u) >= (miss_t[1].reads * 30u));
	CHECK((miss_t[2].right[DEC_PART] * 100u) >= (miss_t[2].reads * 20u));
}

/* ---- CPU ------------------------------------------------------------------ */

static void test_cpu(void)
{
	pm_opt_t  opt;
	uint32_t  cmax = 0u;
	uint32_t  csum = 0u;
	unsigned  calls = 0u;
	unsigned  k;
	clock_t   t0;
	double    us;

	pm_rng = PROP_SEED + 1u;
	pm_opt_default(&opt);
	t0 = clock();
	for (k = 0u; k < 20000u; k++) {
		uint32_t   w[64];
		pm_truth_t t;
		bc_part_t  pt;
		int        n;
		uint32_t   c;

		if ((k % 2u) == 0u) {
			opt.miss = (int)(k % 4u);
			n = pm_read((char)('A' + (k % 26u)), (int)((k / 2u) & 1u), &opt, w, &t);
		} else {
			do {
				n = pm_noise(w, 1 + (int)((k / 2u) % 2u));	/* noise: the most cells */
			} while (n < (int)BC_PART_N_MIN);
		}
		(void)bc_partial_cells(true);
		(void)bc_partial_decode(w, (uint16_t)n, &pt);
		c = bc_partial_cells(false);
		cmax = (c > cmax) ? c : cmax;
		csum += c;
		calls++;
	}
	us = (1e6 * (double)(clock() - t0)) / (double)CLOCKS_PER_SEC / (double)calls;
	(void)printf("CPU: %u calls (half noise), cells mean %u, max %u (limit %u); host %.1f us per call\n",
		     calls, (unsigned)(csum / calls), (unsigned)cmax, (unsigned)BC_PART_CELLS_MAX, us);
	CHECK(cmax <= BC_PART_CELLS_MAX);
}

int main(void)
{
	test_car_reads();
	test_one_lost();
	test_property();
	test_cpu();
	return check_report("test_bc_analog_partial");
}
