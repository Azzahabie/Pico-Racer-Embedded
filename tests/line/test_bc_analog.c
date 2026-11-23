/* Host test for the analog barcode path's pure modules
   (BARCODE_SENSOR_ANALOG 1): the other team's Code 39 decoders as adapted
   (bc_analog_code39.c), their capture as adapted (bc_analog_capture.c) and
   our front end (bc_analog_front.c).  The other team's own host tests were not in
   the files we were given, so these are new; the decoder cases follow the
   behaviour their comments describe.  Sheets come from bc_sim.h (sim).

   Also a false-decode property test: random widths, and valid reads
   damaged, never give a letter other than the one printed (the rates are
   printed). */

#include <stdio.h>
#include <string.h>

#include "check.h"
#include "bc_sim.h"
#include "../../app_program/line_task/bc_analog_front.h"
#include "../../app_program/line_task/bc_analog_capture.h"
#include "../../app_program/motor_task/motion_ds.h"

static const char k_set[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

/* ---- the decoders, on exact widths ---------------------------------- */

/* The widths of *c* for any Code 39 character, in um (n narrow, r ratio). */
static uint16_t code_widths(char c, uint32_t n, uint32_t r10, int rev, uint32_t *w)
{
	char text[3] = { '*', c, '*' };
	uint16_t o = 0u;
	unsigned k;
	unsigned i;

	for (k = 0u; k < 3u; ++k) {
		uint16_t p = code39_pattern(text[k]);

		for (i = 0u; i < 9u; ++i) {
			w[o++] = (((p >> (8u - i)) & 1u) != 0u) ? (n * r10) / 10u : n;
		}
		if (k < 2u) {
			w[o++] = n;
		}
	}
	if (rev) {
		for (i = 0u; i < 14u; ++i) {
			uint32_t t = w[i];

			w[i] = w[28u - i];
			w[28u - i] = t;
		}
	}
	return o;
}

static void test_decoders(void)
{
	uint32_t w[64];
	char ch;
	bool rev;
	unsigned i;
	unsigned bad = 0u;
	int d;

	/* every data character, both ways, ratios 2 and 3 */
	for (i = 0u; i < sizeof(k_set) - 1u; ++i) {
		for (d = 0; d < 2; ++d) {
			uint32_t r10;

			for (r10 = 20u; r10 <= 30u; r10 += 10u) {
				uint16_t n = code_widths(k_set[i], 3400u, r10, d, w);

				if ((code39_decode_one(w, n, BC_RATIO_MIN_X10, &ch, &rev) != CODE39_OK) ||
				    (ch != k_set[i]) || (rev != (d != 0))) {
					bad++;
				}
			}
		}
	}
	CHECK(bad == 0u);
	CHECK(code39_char(code39_pattern('Z')) == 'Z');
	CHECK(code39_bits_ok());
	CHECK(code39_pattern('a') == 0u);

	/* fewer than 9 elements: not a barcode (a crossing line) */
	w[0] = 18000u;
	CHECK(code39_decode_one_repair(w, 1u, BC_RATIO_MIN_X10, &ch, &rev) == CODE39_NOT_BARCODE);

	/* The other team's repair: one bar and its space lost at either end, either way */
	for (d = 0; d < 4; ++d) {
		uint16_t n = code_widths('A', 3400u, 25u, d & 1, w);

		if (d < 2) {
			memmove(w, &w[2], (size_t)(n - 2u) * sizeof(w[0]));
		}
		n = (uint16_t)(n - 2u);
		ch = '\0';
		CHECK(code39_decode_one(w, n, BC_RATIO_MIN_X10, &ch, &rev) != CODE39_OK);
		CHECK(code39_decode_one_repair(w, n, BC_RATIO_MIN_X10, &ch, &rev) == CODE39_OK);
		CHECK(ch == 'A');
	}

	/* ours: two pairs lost at one end (BC_REPAIR4) */
	for (d = 0; d < 4; ++d) {
		uint16_t n = code_widths('Z', 5100u, 25u, d & 1, w);
		code39_result_t r;

		if (d < 2) {
			memmove(w, &w[4], (size_t)(n - 4u) * sizeof(w[0]));
		}
		n = (uint16_t)(n - 4u);
		ch = '\0';
		r = code39_decode_one_repair(w, n, BC_RATIO_MIN_X10, &ch, &rev);
#if BC_REPAIR4
		CHECK(r == CODE39_OK);
		CHECK(ch == 'Z');
#else
		CHECK(r != CODE39_OK);
#endif
	}

	/* The other team's best match: a thick bar split by a speck (31 elements) */
	{
		uint16_t n = code_widths('K', 3400u, 25u, 0, w);
		uint32_t s[64];
		uint16_t k;
		uint16_t o = 0u;

		/* element 12 is the data character's 3rd element (index 2) */
		for (k = 0u; k < n; ++k) {
			if (k == 14u) {	/* a bar of K */
				s[o++] = w[k] / 2u - 600u;
				s[o++] = 1200u;		/* the speck: a light gap */
				s[o++] = w[k] / 2u - 600u;
			} else {
				s[o++] = w[k];
			}
		}
		ch = '\0';
		CHECK(code39_decode_one(s, o, BC_RATIO_MIN_X10, &ch, &rev) != CODE39_OK);
		CHECK(code39_decode_one_best(s, o, &ch, &rev) == CODE39_OK);
		CHECK(ch == 'K');
	}
}

/* ---- false decodes ----------------------------------------------------- */

static uint32_t rnd(void)
{
	bc_sim_rand = bc_sim_rand * 1103515245u + 12345u;
	return bc_sim_rand >> 8;
}

/* Every decoder a read can reach in the other team's bc_finish(), in their order. */
static int decode_any(uint32_t *w, uint16_t n, char *ch)
{
	bool rev;

	if (code39_decode_one_repair(w, n, BC_RATIO_MIN_X10, ch, &rev) == CODE39_OK) {
		return 1;
	}
	if (code39_decode_one_best(w, n, ch, &rev) == CODE39_OK) {
		return 2;
	}
	return 0;
}

static void test_false_decodes(void)
{
	uint32_t w[64];
	char ch;
	unsigned trial;
	unsigned random_letters = 0u;
	unsigned random_n = 0u;
	unsigned dmg_same = 0u;
	unsigned dmg_wrong = 0u;
	unsigned dmg_none = 0u;
	unsigned dmg_n = 0u;
	unsigned swap_n = 0u;
	unsigned swap_other = 0u;	/* the swap made another valid code */

	/* random reads: 9-59 elements of 1-20 mm, and of only two widths
	   (narrow, 2-3 x wide), which look most like a code */
	for (trial = 0u; trial < 200000u; ++trial) {
		uint16_t n = (uint16_t)(9u + (rnd() % 51u));
		uint16_t k;
		bool two = (trial % 2u) != 0u;
		uint32_t nar = 2000u + (rnd() % 5000u);

		if ((trial % 4u) < 2u) {
			n = (uint16_t)(23u + (rnd() % 13u));	/* near 29 */
		}
		for (k = 0u; k < n; ++k) {
			w[k] = two ? (((rnd() % 3u) == 0u) ? (nar * (20u + (rnd() % 11u))) / 10u : nar)
				   : 1000u + (rnd() % 19000u);
		}
		random_n++;
		if (decode_any(w, n, &ch) != 0) {
			random_letters++;
		}
	}
	/* valid reads damaged once: an element doubled, halved, removed, a
	   neighbour pair merged, or a narrow and a wide element swapped */
	for (trial = 0u; trial < 60000u; ++trial) {
		char c = k_set[rnd() % (sizeof(k_set) - 1u)];
		uint16_t n = code_widths(c, 3000u + (rnd() % 4000u), 20u + (rnd() % 11u),
					 (int)(rnd() % 2u), w);
		uint16_t at = (uint16_t)(rnd() % n);
		uint16_t k;
		uint32_t t;

		for (k = 0u; k < n; ++k) {	/* +-15 % width noise */
			w[k] = (w[k] * (85u + (rnd() % 31u))) / 100u;
		}
		unsigned kind = rnd() % 5u;

		switch (kind) {
		case 0u:
			w[at] *= 2u;
			break;
		case 1u:
			w[at] /= 2u;
			break;
		case 2u:
			memmove(&w[at], &w[at + 1u], (size_t)(n - at - 1u) * sizeof(w[0]));
			n--;
			break;
		case 3u:
			if ((at + 2u) < n) {
				w[at] += w[at + 1u] + w[at + 2u];
				memmove(&w[at + 1u], &w[at + 3u], (size_t)(n - at - 3u) * sizeof(w[0]));
				n = (uint16_t)(n - 2u);
			}
			break;
		default:
			k = (uint16_t)(rnd() % n);
			if (((k ^ at) & 1u) == 0u) {
				t = w[at];
				w[at] = w[k];
				w[k] = t;
			}
			break;
		}
		ch = '\0';
		if (kind == 4u) {
			/* two elements of a kind swapped: often simply another
			   valid code, which no decoder can tell (reported) */
			swap_n++;
			swap_other += ((decode_any(w, n, &ch) != 0) && (ch != c)) ? 1u : 0u;
			continue;
		}
		dmg_n++;
		if (decode_any(w, n, &ch) == 0) {
			dmg_none++;
		} else if (ch == c) {
			dmg_same++;
		} else {
			dmg_wrong++;
		}
	}
	(void)printf("false decodes: random reads %u letters in %u (%.4f %%); reads with an element "
		     "doubled, halved, lost or merged %u: %u the printed letter, %u none, %u another "
		     "letter (%.4f %%); two elements swapped %u: %u another letter (another valid code)\n",
		     random_letters, random_n, 100.0 * random_letters / random_n, dmg_n,
		     dmg_same, dmg_none, dmg_wrong, 100.0 * dmg_wrong / dmg_n, swap_n, swap_other);
	/* bounds, not zero */
	CHECK((random_letters * 10000u) < random_n);		/* under 0.01 % */
	CHECK((dmg_wrong * 1000u) < dmg_n);			/* under 0.1 %  */
}

/* ---- the other team's original run merge, to compare with ours ------------------ */
/* Copied unchanged from the other team's line_barcode.c but for the names: the
   adapted runs_to_widths() is one pass; both must give the same widths. */
static uint16_t ref_runs_to_widths(uint16_t * p_runs, uint16_t nr, uint32_t * p_widths, uint16_t max)
{
    uint16_t k = 0u;
    uint16_t i;
    uint16_t out = 0u;

    while (k < nr)
    {
        if ((p_runs[k] < BC_PROF_MIN_BINS) && (k > 0u) && ((k + 1u) < nr))
        {
            p_runs[k - 1u] = (uint16_t)(p_runs[k - 1u] + p_runs[k] + p_runs[k + 1u]);
            for (i = k; (i + 2u) < nr; i++)
            {
                p_runs[i] = p_runs[i + 2u];
            }
            nr = (uint16_t)(nr - 2u);
        }
        else
        {
            k++;
        }
    }
    for (k = 0u; (k < nr) && (out < max); k++)
    {
        p_widths[out] = (uint32_t)p_runs[k] * BC_PROF_BIN_UM;
        out++;
    }
    return out;
}

static uint16_t ref_prof_elements(uint16_t const * p_prof, uint16_t n_bins, uint16_t thr, uint16_t hyst,
                          uint32_t * p_widths, uint16_t max)
{
    static uint16_t runs_e[BC_PROF_BINS / 2u];
    uint16_t nr = 0u;
    uint16_t i = 0u;
    uint16_t run = 0u;
    bool     b_black = true;

    /* skip to the first bar */
    while ((i < n_bins) && (p_prof[i] < (uint16_t)(thr + hyst)))
    {
        i++;
    }
    for (; i < n_bins; i++)
    {
        bool const b_now = b_black ? (p_prof[i] + hyst >= thr) : (p_prof[i] >= (uint16_t)(thr + hyst));

        if ((b_now != b_black) && (run > 0u))
        {
            if (nr < (uint16_t)(sizeof(runs_e) / sizeof(runs_e[0])))
            {
                runs_e[nr] = run;
                nr++;
            }
            run     = 0u;
            b_black = b_now;
        }
        run++;
    }
    if (b_black && (run > 0u) && (nr < (uint16_t)(sizeof(runs_e) / sizeof(runs_e[0]))))
    {
        runs_e[nr] = run;                 /* a bar ran to the end: keep it  */
        nr++;
    }
    return ref_runs_to_widths(runs_e, nr, p_widths, max);
}

static uint16_t ref_prof_swing(uint16_t const * p_prof, uint16_t n_bins, uint16_t delta,
                       uint32_t * p_widths, uint16_t max)
{
    static uint16_t runs_s[BC_PROF_BINS / 2u];
    uint16_t nr = 0u;
    uint16_t i;
    uint16_t run = 0u;
    uint16_t ext;
    bool     b_black = false;
    bool     b_bar_seen = false;

    if (n_bins == 0u)
    {
        return 0u;
    }
    ext = p_prof[0];
    for (i = 0u; i < n_bins; i++)
    {
        uint16_t const v = p_prof[i];
        bool           b_now = b_black;

        if (b_black)
        {
            ext = (v > ext) ? v : ext;
            if (((uint32_t)v + delta) < ext)
            {
                b_now = false;
                ext   = v;
            }
        }
        else
        {
            ext = (v < ext) ? v : ext;
            if ((uint32_t)v > ((uint32_t)ext + delta))
            {
                b_now = true;
                ext   = v;
            }
        }
        if (b_now != b_black)
        {
            if (b_bar_seen && (nr < (uint16_t)(sizeof(runs_s) / sizeof(runs_s[0]))))
            {
                runs_s[nr] = run;
                nr++;
            }
            b_bar_seen = b_bar_seen || b_now;
            run     = 0u;
            b_black = b_now;
        }
        run++;
    }
    if (b_black && (run > 0u) && (nr < (uint16_t)(sizeof(runs_s) / sizeof(runs_s[0]))))
    {
        runs_s[nr] = run;                 /* a bar ran to the end: keep it  */
        nr++;
    }
    return ref_runs_to_widths(runs_s, nr, p_widths, max);
}

static void test_runs_equal(void)
{
    static uint16_t prof[BC_PROF_BINS];
    uint32_t a[BC_MAX_ELEMENTS + 1u];
    uint32_t b[BC_MAX_ELEMENTS + 1u];
    unsigned trial;
    unsigned diff = 0u;

    for (trial = 0u; trial < 4000u; ++trial)
    {
        uint16_t n = (uint16_t)(1u + (rnd() % BC_PROF_BINS));
        uint16_t i;
        uint16_t na;
        uint16_t nb;
        uint16_t thr = (uint16_t)(rnd() % 900u);
        uint16_t hyst = (uint16_t)(rnd() % 80u);
        unsigned lvl = 50u + (rnd() % 950u);
        unsigned run = 1u + (rnd() % 6u);

        for (i = 0u; i < n; ++i)
        {
            if ((rnd() % run) == 0u)
            {
                lvl = rnd() % 1001u;
            }
            prof[i] = (uint16_t)lvl;
        }
        na = ref_prof_elements(prof, n, thr, hyst, a, (uint16_t)BC_MAX_ELEMENTS);
        nb = bc_prof_elements(prof, n, thr, hyst, b, (uint16_t)BC_MAX_ELEMENTS);
        diff += ((na != nb) || (memcmp(a, b, (size_t)na * sizeof(a[0])) != 0)) ? 1u : 0u;
        na = ref_prof_swing(prof, n, (uint16_t)(60u + thr / 4u), a, (uint16_t)BC_MAX_ELEMENTS);
        nb = bc_prof_swing(prof, n, (uint16_t)(60u + thr / 4u), b, (uint16_t)BC_MAX_ELEMENTS);
        diff += ((na != nb) || (memcmp(a, b, (size_t)na * sizeof(a[0])) != 0)) ? 1u : 0u;
    }
    CHECK(diff == 0u);
}

/* ---- the front end ----------------------------------------------------- */

typedef struct {
	bc_front_t f;
	uint32_t ctr;
	uint32_t word;
	double x;		/* the sensor's position, mm */
	double d;		/* the car's distance, mm */
	unsigned t;		/* ms */
} rig_t;

static void rig_init(rig_t *g)
{
	bc_front_init(&g->f);
	g->ctr = 0u;
	g->word = motion_ds_pack(100, MOTION_DS_STEADY, 0u, 0);
	g->x = 0.0;
	g->d = 100.0;
	g->t = 0u;
}

/* One 1 ms sample at v mm/s: motion publishes every 10 ms (whole mm). */
static uint32_t rig_step(rig_t *g, double v_mm_s, uint16_t raw)
{
	g->x += v_mm_s / 1000.0;
	g->d += ((v_mm_s < 0.0) ? -v_mm_s : v_mm_s) / 1000.0 * ((v_mm_s < 0.0) ? -1.0 : 1.0);
	if ((g->t % 10u) == 0u) {
		g->ctr++;
		g->word = motion_ds_pack((int32_t)g->d, MOTION_DS_STEADY, g->ctr, 0);
	}
	g->t++;
	bc_front_step(&g->f, raw, g->word, true, 1u);
	return bc_front_take_step(&g->f);
}

static void test_front(void)
{
	static const double speeds[] = { 15.0, 32.0, 47.6, 100.0 };
	rig_t g;
	unsigned i;
	unsigned k;
	double sum;
	uint32_t s;

	/* steady speed: the travel over 2 s is the car's within 2 % */
	for (i = 0u; i < 4u; ++i) {
		rig_init(&g);
		for (k = 0u; k < 1000u; ++k) {
			(void)rig_step(&g, speeds[i], 200u);
		}
		sum = 0.0;
		for (k = 0u; k < 2000u; ++k) {
			sum += (double)rig_step(&g, speeds[i], 200u);
		}
		CHECK((sum / 1000.0 > speeds[i] * 2.0 * 0.98) && (sum / 1000.0 < speeds[i] * 2.0 * 1.02));
	}
	/* the car stops: the steps fall to 0 within BC_FRONT_STOP_MS + 10 */
	rig_init(&g);
	for (k = 0u; k < 1000u; ++k) {
		(void)rig_step(&g, 32.0, 200u);
	}
	for (k = 0u; k < BC_FRONT_STOP_MS + 10u; ++k) {
		s = rig_step(&g, 0.0, 200u);
	}
	CHECK(s == 0u);
	CHECK(bc_front_speed_mm_s(&g.f) == 0u);
	/* reversing is travel too */
	rig_init(&g);
	sum = 0.0;
	for (k = 0u; k < 2000u; ++k) {
		sum += (double)rig_step(&g, -32.0, 200u);
	}
	CHECK(sum > 50000.0);
	/* no distance_state: BC_ANALOG_NO_DS_UM_PER_MS */
	bc_front_init(&g.f);
	bc_front_step(&g.f, 200u, 0u, false, 1u);
	CHECK(bc_front_take_step(&g.f) == BC_ANALOG_NO_DS_UM_PER_MS);
	/* stuck counter: the same after LINE_SCAN_STALE_MS */
	bc_front_init(&g.f);
	for (k = 0u; k < LINE_SCAN_STALE_MS + 5u; ++k) {
		bc_front_step(&g.f, 200u, 1234u, true, 1u);
	}
	CHECK(!g.f.ds_ok);

	/* the floor-relative level: white 200, black from +ON_DELTA, white
	   again within +OFF_DELTA, a short blip filtered */
	bc_front_init(&g.f);
	for (k = 0u; k < 100u; ++k) {
		bc_front_step(&g.f, 200u, 0u, false, 1u);
	}
	CHECK(g.f.floor_v == 200u);
	CHECK(!g.f.black && !g.f.seen && (g.f.norm == 0u));
	for (k = 0u; k < 3u; ++k) {
		bc_front_step(&g.f, (uint16_t)(200u + BC_ANALOG_ON_DELTA), 0u, false, 1u);
	}
	CHECK(g.f.black && !g.f.seen);			/* a 3 ms blip */
	bc_front_step(&g.f, (uint16_t)(200u + BC_ANALOG_OFF_DELTA + 1u), 0u, false, 1u);
	CHECK(g.f.black);				/* hysteresis */
	bc_front_step(&g.f, (uint16_t)(200u + BC_ANALOG_OFF_DELTA), 0u, false, 1u);
	CHECK(!g.f.black && !g.f.seen);
	for (k = 0u; k < BARCODE_GLITCH_TICKS + 1u; ++k) {
		bc_front_step(&g.f, 2500u, 0u, false, 1u);
	}
	CHECK(g.f.seen);
	CHECK(g.f.norm == ((2500u - 200u) * 1000u > 1000u * BC_ANALOG_SPAN_RAW ? 1000u
			   : ((2500u - 200u) * 1000u) / BC_ANALOG_SPAN_RAW));
	/* the floor holds over black, creeps up over white */
	for (k = 0u; k < 2000u; ++k) {
		bc_front_step(&g.f, 2500u, 0u, false, 1u);
	}
	CHECK(g.f.floor_v == 200u);
	for (k = 0u; k < 80u; ++k) {
		bc_front_step(&g.f, 260u, 0u, false, 1u);
	}
	CHECK(g.f.floor_v == 210u);			/* 80 ms / 8 */
	bc_front_step(&g.f, 150u, 0u, false, 1u);
	CHECK(g.f.floor_v == 150u);			/* down at once */
}

/* ---- the capture on sheets from the model ------------------------------ */

typedef struct {
	char letter;
	unsigned valid_early;	/* BC_VALID from bc_sample (stop END)   */
	unsigned valid_late;	/* BC_VALID from bc_finish              */
	unsigned rejects;
	unsigned not_barcode;
	uint8_t pass;
} cap_res_t;

static void drive_capture(rig_t *g, bc_capture_t *bc, double x1, double v, cap_res_t *res)
{
	bc_out_t o;

	while (g->x < x1) {
		uint32_t step = rig_step(g, v, bc_sim_read(g->x));
		bc_result_t r = bc_sample(bc, g->f.norm, step, &o);

		if (r == BC_VALID) {
			res->valid_early++;
			res->letter = (res->letter == '\0') ? o.ch : res->letter;
			res->pass = o.pass;
		} else if (r == BC_DONE) {
			r = bc_finish(bc, &o);
			bc_resume(bc, r);
			if (r == BC_VALID) {
				res->valid_late++;
				res->letter = (res->letter == '\0') ? o.ch : res->letter;
				res->pass = o.pass;
			} else if (r == BC_REJECT) {
				res->rejects++;
			} else {
				res->not_barcode++;
			}
		} else {
			/* nothing */
		}
	}
}

static void test_capture(void)
{
	static const double narrow[3] = { 3.4, 5.1, 6.9 };	/* the course's scales */
	static const double ratio[2] = { 2.5, 3.0 };
	rig_t g;
	bc_capture_t bc;
	cap_res_t res;
	unsigned bad = 0u;
	unsigned late = 0u;
	unsigned runs = 0u;
	unsigned s;
	unsigned q;
	int d;
	char c;
	double end;

	for (s = 0u; s < 3u; ++s) {
		for (q = 0u; q < 2u; ++q) {
			for (d = 0; d < 2; ++d) {
				for (c = 'A'; c <= 'Z'; ++c) {
					rig_init(&g);
					bc_init(&bc);
					bc_sim_clear();
					end = bc_sim_sheet(c, narrow[s], ratio[q], d, 60.0, BC_SIM_BLACK);
					(void)memset(&res, 0, sizeof(res));
					drive_capture(&g, &bc, end + 80.0, 32.0, &res);
					runs++;
					if ((res.letter != c) || (res.rejects != 0u) ||
					    ((res.valid_early + res.valid_late) != 1u)) {
						bad++;
					}
					late += res.valid_late;
				}
			}
		}
	}
	(void)printf("capture: %u sheets (3 scales, ratio 2.5/3, both ways): %u wrong, "
		     "%u decoded after the other team's end instead of at the stop\n", runs, bad, late);
	CHECK(bad == 0u);
	CHECK(late == 0u);

	/* faint thin bars: the stop END cannot see them, the other team's re-reads of
	   the darkness profile can, while the white between thick bars stays
	   under their 40 mm end of a code (5.1 mm scale: up to 43 mm, so a
	   sheet whose thin bars are all unseen splits).  700 (500 over white,
	   under the swing's 575 at span 2300; less under a 4 mm spot on a
	   3.4 mm bar) is reported only; never a wrong letter */
	{
		static const double levels[3] = { 700.0, 900.0, 1200.0 };
		unsigned l;

		for (s = 0u; s < 3u; ++s) {
			for (l = 0u; l < 3u; ++l) {
				unsigned faint_ok = 0u;
				unsigned faint_wrong = 0u;

				for (c = 'A'; c <= 'Z'; ++c) {
					rig_init(&g);
					bc_init(&bc);
					bc_sim_clear();
					end = bc_sim_sheet(c, narrow[s], 2.5, (int)(c & 1), 60.0, levels[l]);
					(void)memset(&res, 0, sizeof(res));
					drive_capture(&g, &bc, end + 120.0, 32.0, &res);
					faint_ok += (res.letter == c) ? 1u : 0u;
					faint_wrong += ((res.letter != '\0') && (res.letter != c)) ? 1u : 0u;
				}
				(void)printf("capture: thin bars at %.0f (white 200, thick 2600), narrow %.1f mm: "
					     "%2u of 26 decode, %u wrong\n", levels[l], narrow[s], faint_ok,
					     faint_wrong);
				CHECK(faint_wrong == 0u);
				if (l > 0u) {		/* 700: reported only */
					CHECK(faint_ok >= (((s == 0u) && (l == 1u)) ? 17u : 24u));
				}
			}
		}
	}

	/* a crossbar alone: not a barcode, no reject */
	rig_init(&g);
	bc_init(&bc);
	bc_sim_clear();
	bc_sim_black(60.0, 78.0, BC_SIM_BLACK);
	(void)memset(&res, 0, sizeof(res));
	drive_capture(&g, &bc, 160.0, 32.0, &res);
	CHECK((res.letter == '\0') && (res.rejects == 0u) && (res.not_barcode == 1u));
	CHECK(bc.state == (uint8_t)BC_ST_ARMED);

	/* the run-on last bar: the sheet's last bar and a crossbar as one */
	rig_init(&g);
	bc_init(&bc);
	bc_sim_clear();
	end = bc_sim_sheet('W', 5.1, 2.5, 0, 60.0, BC_SIM_BLACK);
	bc_sim_black(end, end + 18.0, BC_SIM_BLACK);
	(void)memset(&res, 0, sizeof(res));
	drive_capture(&g, &bc, end + 14.0, 32.0, &res);
	CHECK((res.letter == 'W') && (res.valid_early == 1u));
	CHECK(bc.b_black);		/* still on the crossbar: arm at once */

	/* the scan gate shut: no capture, counted, and the sheet's rest
	   opens none either until a quiet zone */
	rig_init(&g);
	bc_init(&bc);
	bc_sim_clear();
	end = bc_sim_sheet('A', 3.4, 3.0, 0, 60.0, BC_SIM_BLACK);
	(void)memset(&res, 0, sizeof(res));
	bc_set_gate(&bc, false);
	drive_capture(&g, &bc, 62.0, 32.0, &res);
	bc_set_gate(&bc, true);
	drive_capture(&g, &bc, end + 80.0, 32.0, &res);
	CHECK(bc.ignored == 1u);
	CHECK((res.letter == '\0') && (res.rejects == 0u));

	/* a junction on a crossbar that started a capture: dropped */
	rig_init(&g);
	bc_init(&bc);
	bc_sim_clear();
	bc_sim_black(60.0, 78.0, BC_SIM_BLACK);
	(void)memset(&res, 0, sizeof(res));
	drive_capture(&g, &bc, 70.0, 32.0, &res);
	CHECK(bc.state == (uint8_t)BC_ST_CAPTURING);
	CHECK(bc_in_read(&bc) == false);		/* the first bar */
	CHECK(bc_junction(&bc) == BC_NONE);
	CHECK(bc.state == (uint8_t)BC_ST_WAIT_QUIET);

	/* a junction after elements: cut, judge only, then COOLDOWN */
	rig_init(&g);
	bc_init(&bc);
	bc_sim_clear();
	bc_sim_black(60.0, 62.0, BC_SIM_BLACK);	/* a 2 mm speck, then a long black */
	bc_sim_black(70.0, 110.0, BC_SIM_BLACK);
	(void)memset(&res, 0, sizeof(res));
	drive_capture(&g, &bc, 75.0, 32.0, &res);
	CHECK(bc.state == (uint8_t)BC_ST_CAPTURING);
	CHECK(bc.n == 2u);
	CHECK(bc_in_read(&bc));			/* 5 mm: may be a bar */
	drive_capture(&g, &bc, 105.0, 32.0, &res);
	CHECK(bc_in_read(&bc) == false);		/* 35 mm: over 7 x the speck */
	CHECK(bc_junction(&bc) == BC_DONE);
	CHECK(bc.b_cut && (bc.n == 1u) && (bc.state == (uint8_t)BC_ST_DECODING));
	{
		bc_out_t o;
		bc_result_t r = bc_finish(&bc, &o);

		CHECK(r == BC_NONE);
		bc_resume(&bc, r);
		CHECK(bc.state == (uint8_t)BC_ST_COOLDOWN);
	}

	/* bars of an open read are in it; a mission abort drops it */
	rig_init(&g);
	bc_init(&bc);
	bc_sim_clear();
	end = bc_sim_sheet('B', 5.1, 2.5, 0, 60.0, BC_SIM_BLACK);
	(void)memset(&res, 0, sizeof(res));
	drive_capture(&g, &bc, 100.0, 32.0, &res);
	CHECK(bc_busy(&bc));
	CHECK(bc_abort(&bc));
	CHECK(!bc_busy(&bc) && (bc.state == (uint8_t)BC_ST_WAIT_QUIET));
	CHECK(!bc_abort(&bc));
	(void)end;
}

int main(void)
{
	test_decoders();
	test_false_decodes();
	test_runs_equal();
	test_front();
	test_capture();
	return check_report("test_bc_analog");
}
