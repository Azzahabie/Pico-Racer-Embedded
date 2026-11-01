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

/* One tick through capture -> message -> assembler -> verdict. */
static void tick(bool black)
{
	uint8_t msg[BAR_MSG_SIZE];
	uint16_t width = 0u;
	bar_event_t ev = bar_capture_step(&cap, black, &width);
	barcode_rx_event_t rev;

	if (ev == BAR_EV_NONE) {
		return;
	}
	if (ev == BAR_EV_WIDTH) {
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
	ticks(800u, false);
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

	/* W_long: one bar too long (the car stopped on it) */
	reset();
	ticks(10u, false);
	ticks(BARCODE_W_LONG_TICKS + 1u, true);
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
	ticks(BARCODE_Q_TICKS, false);
	CHECK(!bar_capture_busy(&cap));
	/* and the next barcode reads cleanly, under a new read_id */
	scan("K", 10u, 30u, 1);
	CHECK(verdict.accepted && (verdict.letter == 'K'));
	CHECK(cap.read_id != first_id);

	/* W_max: too many widths */
	reset();
	ticks(10u, false);
	for (i = 0u; i < BARCODE_W_MAX + 2u; ++i) {
		ticks(3u, (i % 2u) == 0u);
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
	test_stale_and_overflow();
	return check_report("barcode pipeline");
}
