/* Host test for the barcode path, sample by sample: bar_capture.c (1 ms
   ticks, END/ABORT) -> the 4-byte bar_mbf message -> barcode_rx.c
   (assembly, stale reads) -> barcode_decode.c (ratio decode) -> the table. */

#include <string.h>

#include "check.h"
#include "../../app_program/line_task/bar_capture.h"
#include "../../app_program/line_task/barcode_rx.h"

static const char *const patterns[27] = {
	"WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",
	"WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",
	"NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",
	"WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",
	"NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",
	"WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",
	"WWNNWNNNN", "NWWNWNNNN", "NWNNWNWNN"	/* '*' */
};

#define MAX_ELEMS (BARCODE_MAX_ELEMENTS)

static bar_capture_t cap;
static barcode_rx_t rx;
static barcode_table_t table;
static barcode_verdict_t verdict;
static unsigned verdicts;
static unsigned widths_sent;
static uint16_t sent[8];		/* the first widths of the read */

/* Code 39 element widths (ticks) for text, start/stop added. */
static unsigned elements(const char *text, unsigned narrow, unsigned wide,
			 int reverse, uint16_t out[MAX_ELEMS])
{
	uint16_t tmp[MAX_ELEMS];
	unsigned n = 0u;
	unsigned len = (unsigned)strlen(text);
	unsigned s;
	unsigned i;

	for (s = 0u; s < len + 2u; ++s) {
		char c = ((s == 0u) || (s == len + 1u)) ? '*' : text[s - 1u];
		const char *p = patterns[(c == '*') ? 26 : (c - 'A')];

		for (i = 0u; i < 9u; ++i) {
			tmp[n++] = (uint16_t)((p[i] == 'W') ? wide : narrow);
		}
		if (s + 1u < len + 2u) {
			tmp[n++] = (uint16_t)narrow;	/* inter-character gap */
		}
	}
	for (i = 0u; i < n; ++i) {
		out[i] = reverse ? tmp[n - 1u - i] : tmp[i];
	}
	return n;
}

/* One capture event -> message -> assembler -> verdict. */
static void deliver(bar_event_t ev, uint16_t width)
{
	uint8_t msg[BAR_MSG_SIZE];
	barcode_rx_event_t rev;

	if (ev == BAR_EV_NONE) {
		return;
	}
	if (ev == BAR_EV_WIDTH) {
		if (widths_sent < 8u) {
			sent[widths_sent] = width;
		}
		widths_sent++;
	}
	bar_msg_pack(msg, ev, cap.read_id,
		     (ev == BAR_EV_WIDTH) ? width :
		     (ev == BAR_EV_END) ? cap.count : (uint16_t)cap.reason);
	rev = barcode_rx_feed(&rx, msg);
	if (rev != BARCODE_RX_NONE) {
		barcode_rx_evaluate(&rx, rev, &table, &verdict);
		verdicts++;
	}
}

/* One tick through capture -> message -> assembler -> verdict. */
static void tick(bool black)
{
	uint16_t width = 0u;
	bar_event_t ev = bar_capture_step(&cap, black, &width);

	deliver(ev, width);
}

static void ticks(unsigned n, bool black)
{
	while (n-- > 0u) {
		tick(black);
	}
}

/* Pass a barcode under the sensor: quiet white, the elements, quiet white. */
static void scan(const char *text, unsigned narrow, unsigned wide, int reverse)
{
	uint16_t e[MAX_ELEMS];
	unsigned n = elements(text, narrow, wide, reverse, e);
	unsigned i;

	ticks(100u, false);
	for (i = 0u; i < n; ++i) {
		ticks(e[i], (i % 2u) == 0u);
	}
	ticks(4u * wide + BARCODE_Q_TICKS + 10u, false);
}

static void reset(void)
{
	bar_capture_init(&cap);
	barcode_rx_init(&rx);
	verdicts = 0u;
	widths_sent = 0u;
	(void)memset(&verdict, 0, sizeof(verdict));
}

static void load_table(void)
{
	/* A..Z cycle FORWARD, LEFT, RIGHT, U_TURN: exercises all four */
	static const char text[] = "FLRUFLRUFLRUFLRUFLRUFLRUFL";

	barcode_table_init(&table);
	CHECK(barcode_table_from_string(&table, text));
}

static void test_all_letters_both_ways(void)
{
	static const unsigned narrows[] = { 4u, 10u, 25u };
	char text[2] = { 'A', '\0' };
	unsigned k;
	int reverse;

	load_table();
	for (k = 0u; k < sizeof(narrows) / sizeof(narrows[0]); ++k) {
		for (reverse = 0; reverse <= 1; ++reverse) {
			for (text[0] = 'A'; text[0] <= 'Z'; ++text[0]) {
				unsigned wide = (k == 1u) ? (narrows[k] * 5u) / 2u
							  : narrows[k] * 3u;
				reset();
				scan(text, narrows[k], wide, reverse);
				CHECK(verdicts == 1u);
				CHECK(widths_sent == 29u);
				CHECK(verdict.accepted);
				CHECK(verdict.letter == text[0]);
				CHECK(verdict.decode.reversed == (reverse != 0));
				CHECK((uint32_t)verdict.action ==
				      (uint32_t)(text[0] - 'A') % 4u);
				CHECK(barcode_rx_idle(&rx));
			}
		}
	}
}

static void test_rejects(void)
{
	static const uint16_t junk[29] = {
		10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
		10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10
	};
	unsigned i;

	/* unknown: no table loaded, so no letter maps to a command */
	barcode_table_init(&table);
	reset();
	scan("C", 10u, 30u, 0);
	CHECK((verdicts == 1u) && !verdict.accepted);
	CHECK(verdict.reject == BARCODE_REJECT_UNKNOWN);
	CHECK(verdict.letter == 'C');

	load_table();

	/* malformed: 29 equal widths (no wide elements) */
	reset();
	ticks(100u, false);
	for (i = 0u; i < 29u; ++i) {
		ticks(junk[i], (i % 2u) == 0u);
	}
	ticks(200u, false);
	CHECK((verdicts == 1u) && !verdict.accepted);
	CHECK(verdict.reject == BARCODE_REJECT_DECODE);

	/* malformed: a single black stripe (e.g. a junction line) */
	reset();
	ticks(100u, false);
	ticks(150u, true);
	ticks((150u * BARCODE_Q_WIDTH_FACTOR) + 10u, false);	/* its quiet zone */
	CHECK((verdicts == 1u) && (verdict.reject == BARCODE_REJECT_DECODE));
	CHECK(verdict.decode.error == BARCODE_DECODE_BAD_LENGTH);

	/* two letters: decoded, but not a single-letter command */
	reset();
	scan("AB", 10u, 30u, 0);
	CHECK((verdicts == 1u) && (verdict.reject == BARCODE_REJECT_NOT_ONE_LETTER));
}

static void test_aborts(void)
{
	uint8_t first_id;
	unsigned i;

	load_table();

	/* W_long: one bar too long (the car stopped on it); the glitch
	   filter sees it BARCODE_GLITCH_TICKS late */
	reset();
	ticks(10u, false);
	ticks(BARCODE_W_LONG_TICKS + 1u + BARCODE_GLITCH_TICKS, true);
	CHECK((verdicts == 1u) && (verdict.reject == BARCODE_REJECT_ABORTED));
	CHECK(rx.abort_reason == BAR_ABORT_TOO_LONG);
	CHECK(bar_capture_busy(&cap));		/* discarding until quiet */
	CHECK(!bar_capture_reading(&cap));	/* but not holding line/console */
	ticks(5000u, true);			/* parked on black: still so */
	CHECK(!bar_capture_reading(&cap));
	/* the rest of that barcode does not start a read */
	first_id = cap.read_id;
	for (i = 0u; i < 20u; ++i) {
		ticks(10u, false);
		ticks(10u, true);
	}
	CHECK(cap.read_id == first_id);
	ticks(BARCODE_Q_TICKS + BARCODE_GLITCH_TICKS, false);
	CHECK(!bar_capture_busy(&cap));
	/* and the next barcode reads cleanly, under a new read_id */
	scan("K", 10u, 30u, 1);
	CHECK(verdict.accepted && (verdict.letter == 'K'));
	CHECK(cap.read_id != first_id);

	/* W_max: too many widths (each longer than a glitch) */
	reset();
	ticks(10u, false);
	for (i = 0u; i < BARCODE_W_MAX + 2u; ++i) {
		ticks(BARCODE_GLITCH_TICKS + 2u, (i % 2u) == 0u);
	}
	CHECK((verdicts == 1u) && (verdict.reject == BARCODE_REJECT_ABORTED));
	CHECK(rx.abort_reason == BAR_ABORT_TOO_MANY);

	/* the whole read too long */
	reset();
	ticks(10u, false);
	for (i = 0u; (i * 40u) <= BARCODE_READ_MAX_TICKS; ++i) {
		ticks(40u, (i % 2u) == 0u);
	}
	CHECK(verdicts == 1u);
	CHECK(rx.abort_reason == BAR_ABORT_TIMEOUT);

	/* cancelled mid-read (mission change) */
	reset();
	ticks(10u, false);
	ticks(10u, true);
	ticks(10u, false);
	{
		uint8_t msg[BAR_MSG_SIZE];
		CHECK(bar_capture_abort(&cap, BAR_ABORT_CANCELLED) == BAR_EV_ABORT);
		bar_msg_pack(msg, BAR_EV_ABORT, cap.read_id, (uint16_t)BAR_ABORT_CANCELLED);
		CHECK(barcode_rx_feed(&rx, msg) == BARCODE_RX_ABORTED);
		barcode_rx_evaluate(&rx, BARCODE_RX_ABORTED, &table, &verdict);
		CHECK(!verdict.accepted && (verdict.reject == BARCODE_REJECT_ABORTED));
	}
	CHECK(bar_capture_abort(&cap, BAR_ABORT_CANCELLED) == BAR_EV_NONE);	/* idle */
}

/* Elements in ticks, bars and spaces apart (blur: bars wider, spaces
   narrower by the same amount). */
static unsigned elements_bs(const char *text, unsigned bar_n, unsigned bar_w,
			    unsigned sp_n, unsigned sp_w, int reverse,
			    uint16_t out[MAX_ELEMS])
{
	unsigned n = elements(text, 1u, 2u, reverse, out);	/* 1 = N, 2 = W */
	unsigned i;

	for (i = 0u; i < n; ++i) {
		bool bar = (i % 2u) == 0u;

		out[i] = (uint16_t)((out[i] == 2u) ? (bar ? bar_w : sp_w)
						   : (bar ? bar_n : sp_n));
	}
	return n;
}

/* BARCODE_END_ON_STOP: a valid *X* ENDs on the sample after its stop
   character's last bar, not after the quiet zone, so a crossbar close
   behind it cannot join it (31 widths: malformed), even one printed
   against the last bar.  A read that is not valid at 29 widths carries
   on to the quiet zone as before. */
static void test_end_on_stop(void)
{
	uint16_t e[MAX_ELEMS];
	unsigned n;
	unsigned i;
	int reverse;

	load_table();
	for (reverse = 0; reverse <= 1; ++reverse) {
		n = elements("L", 10u, 30u, reverse, e);

		/* the first white sample past the glitch filter ends the last
		   bar (its width), the next one ENDs the read */
		reset();
		ticks(100u, false);
		for (i = 0u; i < n; ++i) {
			ticks(e[i], (i % 2u) == 0u);
		}
		ticks(1u + BARCODE_GLITCH_TICKS, false);
		CHECK(verdicts == 0u);
		CHECK(bar_capture_in_read(&cap));	/* until its END */
		ticks(1u, false);
		CHECK((verdicts == 1u) && verdict.accepted && (verdict.letter == 'L'));
		CHECK(cap.complete && (cap.letter == 'L'));
		CHECK(!bar_capture_busy(&cap));

		/* a crossbar 2 ticks past a glitch after: read alone; the
		   crossbar opens its own */
		reset();
		ticks(100u, false);
		for (i = 0u; i < n; ++i) {
			ticks(e[i], (i % 2u) == 0u);
		}
		ticks(2u + BARCODE_GLITCH_TICKS, false);
		ticks(300u, true);
		CHECK((verdicts == 1u) && verdict.accepted && (verdict.letter == 'L'));
		CHECK(bar_capture_reading(&cap) && (cap.count == 0u));
		CHECK(!bar_capture_in_read(&cap));	/* its first run: may be a junction */

		/* printed against the crossbar: the last bar runs on */
		reset();
		ticks(100u, false);
		for (i = 0u; (i + 1u) < n; ++i) {
			ticks(e[i], (i % 2u) == 0u);
			if ((i > 0u) && ((i + 1u) < n)) {
				CHECK(bar_capture_in_read(&cap));	/* bars arriving */
			}
		}
		ticks(e[n - 1u] + 300u, true);
		CHECK((verdicts == 1u) && verdict.accepted && (verdict.letter == 'L'));
		CHECK(widths_sent == 29u);
		CHECK(bar_capture_reading(&cap) && (cap.count == 0u));
	}

	/* 29 equal widths (not valid), 2 ticks past a glitch, a black: the
	   read goes on, 31 widths, malformed once quiet */
	reset();
	ticks(100u, false);
	for (i = 0u; i < 29u; ++i) {
		ticks(10u, (i % 2u) == 0u);
	}
	ticks(1u + BARCODE_GLITCH_TICKS, false);	/* the 29th width ends */
	CHECK(!cap.complete && bar_capture_reading(&cap));
	CHECK(!bar_capture_in_read(&cap));	/* past 29: that barcode is over */
	ticks(1u, false);
	ticks(20u, true);
	ticks(10u * BARCODE_Q_WIDTH_FACTOR + 50u, false);
	CHECK((verdicts == 1u) && (verdict.reject == BARCODE_REJECT_DECODE));
	CHECK(rx.count == 31u);
}

/* Blur and steering: bars read 0.5 mm wide, spaces 0.5 mm short, and
   GP7's speed 10 % off (the wide space slow, the narrow ones fast) on the
   course's 3 mm / 9 mm sheet at base 6 (63 / 189 ticks, C): bar N 74,
   W 200, space N 48, W 198 ticks.  The wide space is 4.1 times the
   narrowest width, so a quiet zone of 4 x narrowest ended the read
   inside the barcode (J3); 6 x does not. */
static void test_blur_quiet(void)
{
	uint16_t e[MAX_ELEMS];
	char text[2] = { 'A', '\0' };
	unsigned ok = 0u;
	unsigned n;
	unsigned i;
	int reverse;

	load_table();
	for (reverse = 0; reverse <= 1; ++reverse) {
		for (text[0] = 'A'; text[0] <= 'Z'; ++text[0]) {
			n = elements_bs(text, 74u, 200u, 48u, 198u, reverse, e);
			reset();
			ticks(100u, false);
			for (i = 0u; i < n; ++i) {
				ticks(e[i], (i % 2u) == 0u);
			}
			ticks(48u * BARCODE_Q_WIDTH_FACTOR + 10u, false);
			if ((verdicts == 1u) && verdict.accepted && (verdict.letter == text[0])) {
				ok++;
			}
		}
	}
	CHECK(ok == 52u);
}

/* The glitch filter (BARCODE_GLITCH_TICKS): a black or white run that
   short is merged into the runs either side; a longer one is a width,
   kept exactly. */
static void test_glitch_filter(void)
{
	uint16_t e[MAX_ELEMS];
	unsigned n;
	unsigned g;
	unsigned i;
	int reverse;
	uint8_t id;

	load_table();
	/* every bar broken by a white glitch in its middle and every space
	   by a black one, 1 to BARCODE_GLITCH_TICKS ticks: still 29 widths,
	   still the letter */
	for (g = 1u; g <= BARCODE_GLITCH_TICKS; ++g) {
		for (reverse = 0; reverse <= 1; ++reverse) {
			n = elements("M", 10u, 30u, reverse, e);
			reset();
			ticks(100u, false);
			for (i = 0u; i < n; ++i) {
				bool bar = (i % 2u) == 0u;

				ticks(e[i] / 2u, bar);
				ticks(g, !bar);
				ticks(e[i] - (e[i] / 2u) - g, bar);
			}
			ticks(10u * BARCODE_Q_WIDTH_FACTOR + 10u, false);
			CHECK((verdicts == 1u) && verdict.accepted && (verdict.letter == 'M'));
			CHECK(widths_sent == 29u);
		}
	}

	/* one tick longer is a width: bar 20, white, bar 20, all kept */
	reset();
	ticks(100u, false);
	ticks(20u, true);
	ticks(BARCODE_GLITCH_TICKS + 1u, false);
	ticks(20u, true);
	ticks(200u, false);
	CHECK(widths_sent == 3u);
	CHECK((sent[0] == 20u) && (sent[1] == (BARCODE_GLITCH_TICKS + 1u)) && (sent[2] == 20u));

	/* a black speck that short opens no read; one tick longer does */
	reset();
	ticks(100u, false);
	id = cap.read_id;
	ticks(BARCODE_GLITCH_TICKS, true);
	ticks(10u, false);
	CHECK(!bar_capture_busy(&cap) && (cap.read_id == id));
	CHECK(!bar_capture_black(&cap));
	ticks(BARCODE_GLITCH_TICKS + 1u, true);
	CHECK(bar_capture_reading(&cap) && (cap.read_id != id));
	CHECK(bar_capture_black(&cap));
}

/* Feed the read's first widths, then GP7 black: how long the run may get
   before it is not a bar (bar_capture_in_read false). */
static unsigned in_read_for(const uint16_t *w, unsigned nw, unsigned limit)
{
	unsigned i;
	unsigned run = 0u;

	reset();
	ticks(100u, false);
	for (i = 0u; i < nw; ++i) {
		ticks(w[i], (i % 2u) == 0u);
	}
	ticks(BARCODE_GLITCH_TICKS, true);	/* the filter's lag */
	while ((run < limit) && bar_capture_in_read(&cap)) {
		tick(true);
		run++;
	}
	return run;
}

/* BARCODE_NOT_BAR_RATIO_X10: inside an open read, a black run is not a
   bar once it is longer than the ratio times the widest width before the
   space in front of it. */
static void test_not_a_bar(void)
{
	static const uint16_t dropout[] = { 63u, BARCODE_GLITCH_TICKS + 1u };
	static const uint16_t speck[] = { 42u, 105u };
	static const uint16_t star_rev[] = { 10u, 10u };
	static const uint16_t fragment[] = { 10u, 10u, 10u, 10u, 10u, 10u, 10u, 10u };
	uint16_t junk[18];
	uint16_t e[MAX_ELEMS];
	char text[2] = { 'A', '\0' };
	unsigned bad = 0u;
	unsigned n;
	unsigned i;
	unsigned k;
	int reverse;

	/* a dropout 3 mm into the crossbar at base 6 (63 ms, then 4 ms of
	   white): the black after it is not a bar from 2.5 x 63 */
	CHECK(in_read_for(dropout, 2u, 1000u) ==
	      ((BARCODE_NOT_BAR_RATIO_X10 * 63u) / 10u) + 1u);
	/* a 2 mm speck, then 5 mm of white (base 6): from 7 x the speck,
	   since one bar and one space may be a reversed start character */
	CHECK(in_read_for(speck, 2u, 1000u) ==
	      ((BARCODE_NOT_BAR_EARLY_X10 * 42u) / 10u) + 1u);
	/* the reversed start character's wide bar (3 x) is a bar */
	CHECK(in_read_for(star_rev, 2u, 1000u) > 60u);
	/* 8 narrow widths (a read that began part way through a barcode)
	   then a wide bar: still a bar up to 7 x */
	CHECK(in_read_for(fragment, 8u, 1000u) ==
	      ((BARCODE_NOT_BAR_EARLY_X10 * 10u) / 10u) + 1u);
	/* an unreadable sheet: 9 equal bars, then 25 ticks of white before
	   the crossbar; the gap is left out, so 2.5 x the 10-tick bars */
	for (i = 0u; i < 17u; ++i) {
		junk[i] = 10u;
	}
	junk[17] = 25u;
	CHECK(in_read_for(junk, 18u, 1000u) ==
	      ((BARCODE_NOT_BAR_RATIO_X10 * 10u) / 10u) + 1u);

	/* no bar of a real barcode is ever "not a bar": every letter, both
	   ways, wide 2-3 x, bars read wider or narrower than the spaces */
	load_table();
	for (k = 0u; k < 6u; ++k) {
		static const unsigned bn[] = { 10u, 10u, 10u, 13u, 7u, 13u };
		static const unsigned bw[] = { 20u, 25u, 30u, 33u, 27u, 23u };
		static const unsigned sn[] = { 10u, 10u, 10u, 7u, 13u, 7u };
		static const unsigned sw[] = { 20u, 25u, 30u, 27u, 33u, 17u };

		for (reverse = 0; reverse <= 1; ++reverse) {
			for (text[0] = 'A'; text[0] <= 'Z'; ++text[0]) {
				n = elements_bs(text, bn[k], bw[k], sn[k], sw[k], reverse, e);
				reset();
				ticks(100u, false);
				for (i = 0u; i < n; ++i) {
					unsigned t;

					for (t = 0u; t < e[i]; ++t) {
						tick((i % 2u) == 0u);
						if ((cap.count > 0u) && (cap.count < 29u) &&
						    !bar_capture_in_read(&cap)) {
							bad++;
						}
					}
				}
				ticks(BARCODE_Q_TICKS + 200u, false);
			}
		}
	}
	CHECK(bad == 0u);
}

/* bar_capture_junction(): a junction fired while a read is open. */
static void test_junction_end(void)
{
	uint16_t e[MAX_ELEMS];
	uint8_t id;
	unsigned n;
	unsigned i;

	load_table();

	/* no read open: nothing */
	reset();
	ticks(100u, false);
	CHECK(bar_capture_junction(&cap) == BAR_EV_NONE);

	/* the crossbar opened the read itself: dropped */
	reset();
	ticks(100u, false);
	ticks(300u, true);
	deliver(bar_capture_junction(&cap), 0u);
	CHECK((verdicts == 1u) && (verdict.reject == BARCODE_REJECT_ABORTED));
	CHECK(rx.abort_reason == BAR_ABORT_JUNCTION);

	/* a speck and a gap before the crossbar: the read ENDs with those
	   two widths (malformed, printed), and the rest of the crossbar
	   opens no read */
	reset();
	ticks(100u, false);
	ticks(42u, true);
	ticks(105u, false);
	ticks(300u, true);
	id = cap.read_id;
	deliver(bar_capture_junction(&cap), 0u);
	CHECK((verdicts == 1u) && (verdict.reject == BARCODE_REJECT_DECODE));
	CHECK(rx.count == 2u);
	CHECK(!bar_capture_reading(&cap) && bar_capture_busy(&cap));
	ticks(200u, true);
	ticks(BARCODE_Q_TICKS + BARCODE_GLITCH_TICKS, false);
	CHECK(!bar_capture_busy(&cap) && (cap.read_id == id) && (verdicts == 1u));

	/* a complete read due to END is left to END */
	reset();
	n = elements("L", 10u, 30u, 0, e);
	ticks(100u, false);
	for (i = 0u; i < n; ++i) {
		ticks(e[i], (i % 2u) == 0u);
	}
	ticks(1u + BARCODE_GLITCH_TICKS, false);
	CHECK(cap.end_due);
	CHECK(bar_capture_junction(&cap) == BAR_EV_NONE);
	tick(false);
	CHECK((verdicts == 1u) && verdict.accepted && (verdict.letter == 'L'));
}

static void test_stale_and_overflow(void)
{
	uint16_t e[MAX_ELEMS];
	uint8_t msg[BAR_MSG_SIZE];
	unsigned n;
	unsigned i;

	load_table();
	n = elements("R", 10u, 30u, 0, e);

	/* read 5 loses its END (buffer full); read 6 must not inherit it */
	barcode_rx_init(&rx);
	for (i = 0u; i < 12u; ++i) {
		bar_msg_pack(msg, BAR_EV_WIDTH, 5u, e[i]);
		CHECK(barcode_rx_feed(&rx, msg) == BARCODE_RX_NONE);
	}
	CHECK(!barcode_rx_idle(&rx));		/* no map update mid-read */
	for (i = 0u; i < n; ++i) {
		bar_msg_pack(msg, BAR_EV_WIDTH, 6u, e[i]);
		(void)barcode_rx_feed(&rx, msg);
	}
	bar_msg_pack(msg, BAR_EV_END, 6u, (uint16_t)n);
	CHECK(barcode_rx_feed(&rx, msg) == BARCODE_RX_READ);
	barcode_rx_evaluate(&rx, BARCODE_RX_READ, &table, &verdict);
	CHECK(verdict.accepted && (verdict.letter == 'R'));
	CHECK(barcode_rx_idle(&rx));		/* safe point for a map */

	/* an END for widths that never arrived: one rejected read, no command */
	bar_msg_pack(msg, BAR_EV_END, 7u, 29u);
	CHECK(barcode_rx_feed(&rx, msg) == BARCODE_RX_READ);
	barcode_rx_evaluate(&rx, BARCODE_RX_READ, &table, &verdict);
	CHECK(!verdict.accepted && (verdict.reject == BARCODE_REJECT_OVERFLOW));
	CHECK(barcode_rx_idle(&rx));

	/* a width lost on the way (END count disagrees): rejected */
	for (i = 1u; i < n; ++i) {
		bar_msg_pack(msg, BAR_EV_WIDTH, 8u, e[i]);
		(void)barcode_rx_feed(&rx, msg);
	}
	bar_msg_pack(msg, BAR_EV_END, 8u, (uint16_t)n);
	CHECK(barcode_rx_feed(&rx, msg) == BARCODE_RX_READ);
	barcode_rx_evaluate(&rx, BARCODE_RX_READ, &table, &verdict);
	CHECK(!verdict.accepted && (verdict.reject == BARCODE_REJECT_OVERFLOW));

	/* a mission change drops a partial read */
	bar_msg_pack(msg, BAR_EV_WIDTH, 9u, 10u);
	(void)barcode_rx_feed(&rx, msg);
	barcode_rx_discard(&rx);
	CHECK(barcode_rx_idle(&rx));
}

int main(void)
{
	test_all_letters_both_ways();
	test_rejects();
	test_aborts();
	test_end_on_stop();
	test_blur_quiet();
	test_glitch_filter();
	test_not_a_bar();
	test_junction_end();
	test_stale_and_overflow();
	return check_report("barcode pipeline");
}
