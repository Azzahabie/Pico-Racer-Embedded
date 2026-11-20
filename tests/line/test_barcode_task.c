/* Host test for barcode_task.c, the real task loop: a map arriving from
   comms (barcode_map_submit), the boot table line, every action stored
   from a loaded map, and a map sent while a barcode is being read.
   The kernel is faked: tk_rcv_mbf on bar_mbf plays a script (bar_mbf
   messages built by bar_capture.c from 1 ms samples, maps, mission
   changes, quiet 20 ms waits), and the end of the script leaves the task.

   Built four times (tests/line/Makefile): as committed (no compiled-in
   table), with the writeup's example compiled in (-DBARCODE_DEFAULT_TABLE_SET=1,
   "LRFU" + 22 x F: A left, B right, C straight, D U-turn), with a
   compiled-in table that has a wrong character, and with the example and
   the analog barcode path (-DBARCODE_SENSOR_ANALOG=1: its LETTER and
   ANALOG messages, Boon's decode on this side, the reply word). */

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "check.h"
#include <tk/tkernel.h>		/* the motion sim's stand-in */
#ifndef TRUE
#define TRUE	1		/* as the kernel's typedef.h; not in the stand-in */
#define FALSE	0
#endif
#include "../../app_program/line_task/barcode_task.c"

/* ---- fakes: kernel --------------------------------------------------- */
#define BAR_ID	11
#define MAP_ID	12
#define MAP_QUEUE 2	/* map_mbf: "two queued 7-byte maps" (barcode_task.c) */

typedef enum { EV_MSG, EV_MAP, EV_MISSION, EV_WAIT, EV_MARK } ev_kind_t;
typedef struct {
	ev_kind_t kind;
	uint8_t data[BARCODE_MAP_SIZE];
	int len;
} ev_t;

static ev_t g_ev[1024];
static int g_ev_n;
static int g_ev_at;
static jmp_buf g_done;
static ID g_next_id;
static uint8_t g_mapq[MAP_QUEUE][BARCODE_MAP_SIZE];
static int g_mapq_n;
static ER g_submit_er[8];		/* barcode_map_submit results, in order */
static int g_submit_n;
static size_t g_mark_log[8];		/* log length at each EV_MARK */
static uint32_t g_mark_cmd[8];		/* barcode_cmd_word() at each EV_MARK */
static mission_state_t g_mission;
static char g_log[16384];

ID tk_cre_mbf(CONST T_CMBF *p)
{
	(void)p;
	return g_next_id++;
}

ER tk_snd_mbf(ID id, void *msg, INT size, TMO tmo)
{
	(void)tmo;
	if ((MAP_ID != id) || (BARCODE_MAP_SIZE != size)) {
		return E_OBJ;
	}
	if (MAP_QUEUE <= g_mapq_n) {
		return E_TMOUT;			/* TMO_POL on a full buffer */
	}
	memcpy(g_mapq[g_mapq_n], msg, BARCODE_MAP_SIZE);
	g_mapq_n++;
	return E_OK;
}

/* bar_mbf: the script.  One call is one turn of barcode_task's loop. */
INT tk_rcv_mbf(ID id, void *msg, TMO tmo)
{
	(void)tmo;
	if (MAP_ID == id) {
		if (0 == g_mapq_n) {
			return E_TMOUT;
		}
		memcpy(msg, g_mapq[0], BARCODE_MAP_SIZE);
		memmove(g_mapq[0], g_mapq[1], (size_t)(g_mapq_n - 1) * BARCODE_MAP_SIZE);
		g_mapq_n--;
		return BARCODE_MAP_SIZE;
	}
	while (g_ev_at < g_ev_n) {
		const ev_t *e = &g_ev[g_ev_at++];

		switch (e->kind) {
		case EV_MSG:
			memcpy(msg, e->data, BAR_MSG_SIZE);
			return BAR_MSG_SIZE;
		case EV_MAP:			/* comms_task's handle_map() */
			g_submit_er[g_submit_n++ % 8] = barcode_map_submit(e->data, e->len);
			break;
		case EV_MISSION:
			g_mission = (mission_state_t)e->data[0];
			return E_TMOUT;
		case EV_WAIT:
			return E_TMOUT;
		case EV_MARK:
			g_mark_log[e->data[0]] = strlen(g_log);
			g_mark_cmd[e->data[0]] = barcode_cmd_word();
			break;
		}
	}
	longjmp(g_done, 1);			/* script over: leave the task */
}

ER tk_slp_tsk(TMO tmo)
{
	(void)tmo;
	longjmp(g_done, 2);			/* init failed */
}

/* ---- fakes: console, comms, nav -------------------------------------- */
INT tm_printf(const UB *format, ...)
{
	va_list ap;
	size_t len = strlen(g_log);

	va_start(ap, format);
	(void)vsnprintf(g_log + len, sizeof(g_log) - len, (const char *)format, ap);
	va_end(ap);
	return 0;
}

INT tm_sprintf(UB *str, const UB *format, ...)
{
	va_list ap;
	int n;

	va_start(ap, format);
	n = vsprintf((char *)str, (const char *)format, ap);
	va_end(ap);
	return n;
}

static uint32_t g_e_letter[64];		/* E barcode records sent */
static uint32_t g_e_action[64];
static int g_e_n;

ER comms_tele_send(const UB *body, INT length)
{
	uint32_t pos = COMM_W_KIND_BITS;

	CHECK(COMM_W_INTAKE_SIZE_BARCODE == length);
	CHECK(COMM_W_KIND_BARCODE == (body[0] & ((1u << COMM_W_KIND_BITS) - 1u)));
	g_e_letter[g_e_n % 64] = comm_w_get_bits(body, &pos, COMM_W_BARCODE_LETTER_BITS);
	g_e_action[g_e_n % 64] = comm_w_get_bits(body, &pos, COMM_W_BARCODE_ACTION_BITS);
	g_e_n++;
	return E_OK;
}

mission_state_t nav_mission_state(void)
{
	return g_mission;
}

const char *nav_state_name(mission_state_t state)
{
	static const char *const name[] = { "INITIALISING", "CALIBRATING", "READY",
					    "RUNNING", "PAUSED", "FINISHED", "FAULT" };
	return name[state];
}

/* ---- script building ------------------------------------------------- */
static const char *const patterns[27] = {
	"WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",
	"WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",
	"NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",
	"WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",
	"NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",
	"WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",
	"WWNNWNNNN", "NWWNWNNNN", "NWNNWNWNN"	/* '*' */
};

#define NARROW	10u			/* ticks: a 3 mm bar near base 5 */
#define WIDE	30u

static bar_capture_t g_cap;
static ev_t g_read[64];			/* the messages of one read */
static int g_read_n;

static void sample(bool black)
{
	uint16_t width = 0u;
	bar_event_t ev = bar_capture_step(&g_cap, black, &width);

	if (BAR_EV_NONE != ev) {
		g_read[g_read_n].kind = EV_MSG;
		bar_msg_pack(g_read[g_read_n].data, ev, g_cap.read_id,
			     (BAR_EV_WIDTH == ev) ? width :
			     (BAR_EV_END == ev) ? g_cap.count : (uint16_t)g_cap.reason);
		g_read_n++;
	}
}

static void samples(unsigned n, bool black)
{
	while (n-- > 0u) {
		sample(black);
	}
}

/* *letter* under GP7, forwards or backwards: its messages into g_read. */
static void make_read(char letter, int reverse)
{
	uint16_t e[29];
	unsigned n = 0u;
	unsigned s;
	unsigned i;

	for (s = 0u; s < 3u; ++s) {
		const char *p = patterns[(1u == s) ? (unsigned)(letter - 'A') : 26u];

		for (i = 0u; i < 9u; ++i) {
			e[n++] = (uint16_t)(('W' == p[i]) ? WIDE : NARROW);
		}
		if (s < 2u) {
			e[n++] = (uint16_t)NARROW;	/* inter-character gap */
		}
	}
	g_read_n = 0;
	samples(100u, false);
	for (i = 0u; i < n; ++i) {
		samples(e[reverse ? (n - 1u - i) : i], (0u == (i % 2u)));	/* bars first */
	}
	samples(4u * WIDE + BARCODE_Q_TICKS + 10u, false);
}

static void add(ev_kind_t kind, const uint8_t *data, int len)
{
	g_ev[g_ev_n].kind = kind;
	g_ev[g_ev_n].len = len;
	if (NULL != data) {
		memcpy(g_ev[g_ev_n].data, data, (size_t)len);
	}
	g_ev_n++;
}

/* messages from .. to - 1 of the read in g_read */
static void add_read(int from, int to)
{
	int i;

	for (i = from; (i < to) && (i < g_read_n); ++i) {
		g_ev[g_ev_n++] = g_read[i];
	}
}

static void add_whole_read(char letter, int reverse)
{
	make_read(letter, reverse);
	add_read(0, g_read_n);
	add(EV_WAIT, NULL, 0);
}

/* A map as the laptop packs it: mv, then 2 bits a letter, 'A' first. */
static void add_map(uint32_t mv, const char *table)
{
	uint8_t map[BARCODE_MAP_SIZE] = { 0u };
	uint32_t pos = 0u;
	uint32_t i;

	comm_w_put_bits(map, &pos, mv, COMM_W_MAP_MV_BITS);
	for (i = 0u; i < BARCODE_LETTER_COUNT; ++i) {
		comm_w_put_bits(map, &pos, (uint32_t)(strchr("FLRU", table[i]) - "FLRU"),
				COMM_W_MAP_ACT_BITS);
	}
	add(EV_MAP, map, BARCODE_MAP_SIZE);
}

static void add_mission(mission_state_t m)
{
	uint8_t d = (uint8_t)m;

	add(EV_MISSION, &d, 1);
}

static void add_mark(uint8_t id)
{
	add(EV_MARK, &id, 1);
}

static void reset(void)
{
	g_bar_mbfid = 0;
	g_map_mbfid = 0;
	g_cmd = BARCODE_CMD_NONE;
	g_cmd_seq = 0u;
	g_map_version = 0u;
	g_map_loaded = FALSE;
	g_next_id = BAR_ID;
	g_mapq_n = 0;
	g_submit_n = 0;
	g_ev_n = 0;
	g_ev_at = 0;
	g_e_n = 0;
	g_log[0] = '\0';
	g_mission = MS_READY;
	bar_capture_init(&g_cap);
	memset(g_mark_log, 0, sizeof(g_mark_log));
	memset(g_mark_cmd, 0, sizeof(g_mark_cmd));
}

static void run(void)
{
	if (0 == setjmp(g_done)) {
		barcode_task(0, NULL);
	}
}

/* text a is in the log, and before text b */
static bool log_before(const char *a, const char *b)
{
	const char *p_a = strstr(g_log, a);
	const char *p_b = strstr(g_log, b);

	return (NULL != p_a) && (NULL != p_b) && (p_a < p_b);
}

/* text is first in the log after mark id */
static bool after_mark(uint8_t id, const char *text)
{
	const char *p = strstr(g_log, text);

	return (NULL != p) && ((size_t)(p - g_log) >= g_mark_log[id]);
}

/* text is in the log before mark id */
static bool before_mark(uint8_t id, const char *text)
{
	const char *p = strstr(g_log, text);

	return (NULL != p) && ((size_t)(p - g_log) < g_mark_log[id]);
}

static bool cmd_is(uint32_t word, char letter, barcode_action_t action)
{
	return (barcode_cmd_letter(word) == (uint32_t)(letter - 'A')) &&
	       (barcode_cmd_action(word) == (uint32_t)action);
}

/* ---- scenarios ------------------------------------------------------- */

#if !BARCODE_DEFAULT_TABLE_SET
/* No map, no compiled-in table: every letter is unknown, no command is
   stored, and the E record carries letter 31. */
static void no_table(void)
{
	reset();
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_whole_read('A', 0);
	add_mark(0);
	run();
	CHECK(NULL != strstr(g_log, "[barcode] decoder ready, table: none until a map arrives\n"));
	CHECK(NULL != strstr(g_log, "[barcode] REJECTED: 'A' unknown (no barcode map loaded)\n"));
	CHECK(BARCODE_LETTER_NONE == barcode_cmd_letter(g_mark_cmd[0]));
	CHECK(!barcode_cmd_pending(g_mark_cmd[0], 0u));
	CHECK((1 == g_e_n) && (COMM_W_BARCODE_UNREADABLE == g_e_letter[0]) && (0u == g_e_action[0]));
	CHECK(FALSE == barcode_map_loaded());
	CHECK(0u == barcode_map_version());
}
#elif defined(TEST_BAD_TABLE)
/* A compiled-in table with a wrong character: the boot line says so and
   the car behaves as with no table. */
static void bad_table(void)
{
	reset();
	add(EV_WAIT, NULL, 0);
	add_whole_read('A', 0);
	run();
	CHECK(NULL != strstr(g_log, "[barcode] decoder ready, table: none until a map arrives "
			     "(compiled-in table is not 26 of F/L/R/U)\n"));
	CHECK(NULL != strstr(g_log, "[barcode] REJECTED: 'A' unknown (no barcode map loaded)\n"));
	CHECK(BARCODE_LETTER_NONE == barcode_cmd_letter(barcode_cmd_word()));
}
#else
/* The compiled-in table works with no map: A left ... D U-turn. */
static void compiled_table(void)
{
	reset();
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_whole_read('A', 0);
	add_mark(0);
	add_whole_read('B', 1);
	add_mark(1);
	add_whole_read('C', 0);
	add_mark(2);
	add_whole_read('D', 0);
	add_mark(3);
	run();
	CHECK(NULL != strstr(g_log, "[barcode] decoder ready, table: compiled-in default "
			     "LRFUFFFFFFFFFFFFFFFFFFFFFF\n"));
	CHECK(log_before("'A' -> LEFT (dir=fwd), stored for the next junction",
			 "'B' -> RIGHT (dir=rev), stored for the next junction"));
	CHECK(log_before("'B' -> RIGHT", "'C' -> FORWARD (dir=fwd), stored"));
	CHECK(log_before("'C' -> FORWARD", "'D' -> U_TURN (dir=fwd), stored"));
	CHECK(cmd_is(g_mark_cmd[0], 'A', BARCODE_ACT_LEFT));
	CHECK(cmd_is(g_mark_cmd[1], 'B', BARCODE_ACT_RIGHT));
	CHECK(cmd_is(g_mark_cmd[2], 'C', BARCODE_ACT_FORWARD));
	CHECK(cmd_is(g_mark_cmd[3], 'D', BARCODE_ACT_UTURN));
	CHECK(TRUE == barcode_map_loaded());
	CHECK(0u == barcode_map_version());	/* S.mpv 0: no map yet */
	CHECK((4 == g_e_n) && (3u == g_e_letter[3]) && (BARCODE_ACT_UTURN == g_e_action[3]));
}
#endif

/* A map over MQTT, before the run (the day-of procedure): applied at the
   next idle turn of the loop, and each of the four actions is stored from
   it for the next junction, with a new sequence each time. */
static void mqtt_map_every_action(void)
{
	static const char table[] = "UFLRFFFFFFFFFFFFFFFFFFFFFR";

	reset();
	add(EV_WAIT, NULL, 0);
	add_map(2u, table);
	add(EV_WAIT, NULL, 0);
	add_mark(0);
	add_mission(MS_RUNNING);
	add_whole_read('A', 0);
	add_mark(1);
	add_whole_read('B', 0);
	add_mark(2);
	add_whole_read('C', 1);
	add_mark(3);
	add_whole_read('D', 0);
	add_mark(4);
	add_whole_read('Z', 0);
	add_mark(5);
	run();
	CHECK((1 == g_submit_n) && (E_OK == g_submit_er[0]));
	CHECK(log_before("[barcode] decoder ready", "[barcode] map v2 applied: UFLRFFFFFFFFFFFFFFFFFFFFFR\n"));
	CHECK(before_mark(0, "map v2 applied"));	/* before the run starts */
	CHECK(TRUE == barcode_map_loaded());
	CHECK(2u == barcode_map_version());	/* S.mpv */
	CHECK(NULL != strstr(g_log, "[barcode] 'A' -> U_TURN (dir=fwd), stored for the next junction\n"));
	CHECK(NULL != strstr(g_log, "[barcode] 'B' -> FORWARD (dir=fwd), stored for the next junction\n"));
	CHECK(NULL != strstr(g_log, "[barcode] 'C' -> LEFT (dir=rev), stored for the next junction\n"));
	CHECK(NULL != strstr(g_log, "[barcode] 'D' -> RIGHT (dir=fwd), stored for the next junction\n"));
	CHECK(NULL != strstr(g_log, "[barcode] 'Z' -> RIGHT (dir=fwd), stored for the next junction\n"));
	CHECK(BARCODE_LETTER_NONE == barcode_cmd_letter(g_mark_cmd[0]));
	CHECK(cmd_is(g_mark_cmd[1], 'A', BARCODE_ACT_UTURN));
	CHECK(cmd_is(g_mark_cmd[2], 'B', BARCODE_ACT_FORWARD));
	CHECK(cmd_is(g_mark_cmd[3], 'C', BARCODE_ACT_LEFT));
	CHECK(cmd_is(g_mark_cmd[4], 'D', BARCODE_ACT_RIGHT));
	CHECK(cmd_is(g_mark_cmd[5], 'Z', BARCODE_ACT_RIGHT));
	/* each read is a new command for nav: the sequence moves on */
	CHECK(barcode_cmd_seq(g_mark_cmd[2]) != barcode_cmd_seq(g_mark_cmd[1]));
	CHECK(barcode_cmd_pending(g_mark_cmd[2], barcode_cmd_seq(g_mark_cmd[1])));
	/* the E barcode records (S.bar, S.act) */
	CHECK(5 == g_e_n);
	CHECK((0u == g_e_letter[0]) && (BARCODE_ACT_UTURN == g_e_action[0]));
	CHECK((2u == g_e_letter[2]) && (BARCODE_ACT_LEFT == g_e_action[2]));
	CHECK((25u == g_e_letter[4]) && (BARCODE_ACT_RIGHT == g_e_action[4]));
	CHECK(NULL == strstr(g_log, "REJECTED"));
}

/* A map sent mid-run while a barcode is under GP7: it waits until that read
   ends, so the read is decoded with the table it started with; the next
   read uses the new one. */
static void map_during_a_read(void)
{
	reset();
	add(EV_WAIT, NULL, 0);
	add_map(1u, "LRFUFFFFFFFFFFFFFFFFFFFFFF");
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	make_read('B', 0);
	add_read(0, 10);			/* GP7 is over the bars ... */
	add_map(2u, "LUFUFFFFFFFFFFFFFFFFFFFFFF");	/* ... when B is remapped */
	add(EV_WAIT, NULL, 0);
	add(EV_WAIT, NULL, 0);
	add_mark(0);
	add_read(10, g_read_n);			/* the rest of the read */
	add(EV_WAIT, NULL, 0);
	add_mark(1);
	add_whole_read('B', 0);
	add_mark(2);
	run();
	CHECK((2 == g_submit_n) && (E_OK == g_submit_er[0]) && (E_OK == g_submit_er[1]));
	CHECK(NULL != strstr(g_log, "[barcode] map v1 applied: LRFUFFFFFFFFFFFFFFFFFFFFFF\n"));
	CHECK(!before_mark(0, "map v2"));	/* not while the read is open */
	CHECK(after_mark(0, "[barcode] map v2 applied: LUFUFFFFFFFFFFFFFFFFFFFFFF\n"));
	CHECK(2u == barcode_map_version());
	CHECK(log_before("[barcode] 'B' -> RIGHT (dir=fwd), stored for the next junction\n",
			 "[barcode] map v2 applied"));
	CHECK(log_before("[barcode] map v2 applied",
			 "[barcode] 'B' -> U_TURN (dir=fwd), stored for the next junction\n"));
	CHECK(cmd_is(g_mark_cmd[1], 'B', BARCODE_ACT_RIGHT));
	CHECK(cmd_is(g_mark_cmd[2], 'B', BARCODE_ACT_UTURN));
}

/* A map that lands between a barcode and its junction does not rewrite the
   command already stored for that junction. */
static void map_between_read_and_junction(void)
{
	reset();
	add(EV_WAIT, NULL, 0);
	add_map(3u, "LFFFFFFFFFFFFFFFFFFFFFFFFF");
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_whole_read('A', 0);
	add_mark(0);
	add_map(4u, "RFFFFFFFFFFFFFFFFFFFFFFFFF");
	add(EV_WAIT, NULL, 0);
	add_mark(1);
	run();
	CHECK(cmd_is(g_mark_cmd[0], 'A', BARCODE_ACT_LEFT));
	CHECK(after_mark(0, "[barcode] map v4 applied: RFFFFFFFFFFFFFFFFFFFFFFFFF\n"));
	CHECK(g_mark_cmd[1] == g_mark_cmd[0]);	/* same command, same sequence */
	CHECK(4u == barcode_map_version());
}

#if !BARCODE_DEFAULT_TABLE_SET || defined(TEST_BAD_TABLE)
/* Three maps while a read is open: two wait in map_mbf, the third is
   refused (comms prints "not passed on"), and the last one queued wins. */
static void maps_queue_full(void)
{
	reset();
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	make_read('C', 0);
	add_read(0, 5);
	add_map(5u, "FFLFFFFFFFFFFFFFFFFFFFFFFF");
	add_map(6u, "FFRFFFFFFFFFFFFFFFFFFFFFFF");
	add_map(7u, "FFUFFFFFFFFFFFFFFFFFFFFFFF");
	add_read(5, g_read_n);
	add(EV_WAIT, NULL, 0);
	add_mark(0);
	add_whole_read('C', 0);
	run();
	CHECK(3 == g_submit_n);
	CHECK((E_OK == g_submit_er[0]) && (E_OK == g_submit_er[1]) && (E_TMOUT == g_submit_er[2]));
	CHECK(NULL != strstr(g_log, "[barcode] REJECTED: 'C' unknown (no barcode map loaded)\n"));
	CHECK(log_before("map v5 applied", "map v6 applied"));
	CHECK(NULL == strstr(g_log, "map v7"));
	CHECK(6u == barcode_map_version());
	CHECK(NULL != strstr(g_log, "[barcode] 'C' -> RIGHT (dir=fwd), stored for the next junction\n"));
}
#endif

/* barcode_map_submit refuses any length but 7 (comms checks first). */
static void map_wrong_length(void)
{
	static const uint8_t six[6] = { 0x43u, 0x4eu, 0x4eu, 0x4eu, 0x4eu, 0x4eu };

	reset();
	add(EV_MAP, six, 6);
	add(EV_WAIT, NULL, 0);
	run();
	CHECK((1 == g_submit_n) && (E_PAR == g_submit_er[0]));
	CHECK(NULL == strstr(g_log, "applied"));
	CHECK(0u == barcode_map_version());
}

/* The map stays for the next mission; the stored command does not. */
static void map_survives_new_mission(void)
{
	reset();
	add(EV_WAIT, NULL, 0);
	add_map(9u, "UUUUUUUUUUUUUUUUUUUUUUUUUU");
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_whole_read('Q', 0);
	add_mark(0);
	add_mission(MS_PAUSED);
	add_mission(MS_RUNNING);
	add_mark(1);
	add_mission(MS_FINISHED);
	add_mark(2);
	add_mission(MS_READY);
	add_mission(MS_RUNNING);
	add_whole_read('Q', 0);
	add_mark(3);
	run();
	CHECK(cmd_is(g_mark_cmd[0], 'Q', BARCODE_ACT_UTURN));
	CHECK(g_mark_cmd[1] == g_mark_cmd[0]);		/* a pause keeps it */
	CHECK(BARCODE_LETTER_NONE == barcode_cmd_letter(g_mark_cmd[2]));
	CHECK(NULL != strstr(g_log, "[barcode] mission FINISHED: stored command cleared\n"));
	CHECK(cmd_is(g_mark_cmd[3], 'Q', BARCODE_ACT_UTURN));	/* same map */
	CHECK(9u == barcode_map_version());
}

#if BARCODE_SENSOR_ANALOG
/* ---- the analog path (BARCODE_SENSOR_ANALOG 1) ------------------------ */
/* ir_sense_task's capture, for barcode_task (bc_analog_link.h) */
static bc_capture_t g_bc;

const bc_capture_t *line_bc_capture(void)
{
	return &g_bc;
}

/* Feed g_bc a quiet zone, the elements w (um, bar first) at 32 um a
   sample (bars 900, spaces 40 of 1000), then white until it hands over
   (BC_ST_DECODING). */
static void bc_feed(const uint32_t *w, unsigned n)
{
	bc_out_t o;
	unsigned i;
	uint32_t t;

	bc_init(&g_bc);
	for (t = 0u; t < 40000u; t += 32u) {
		(void)bc_sample(&g_bc, 40u, 32u, &o);
	}
	for (i = 0u; i < n; ++i) {
		for (t = 0u; t < w[i]; t += 32u) {
			(void)bc_sample(&g_bc, ((i % 2u) == 0u) ? 900u : 40u, 32u, &o);
		}
	}
	for (t = 0u; (t < 200000u) && (g_bc.state != (uint8_t)BC_ST_DECODING); t += 32u) {
		(void)bc_sample(&g_bc, 40u, 32u, &o);
	}
	CHECK(g_bc.state == (uint8_t)BC_ST_DECODING);
}

/* *letter* in um (narrow 4 mm, wide 10 mm), its first `drop` elements
   lost; returns the count */
static unsigned bc_letter(char letter, unsigned drop, uint32_t *w)
{
	unsigned n = 0u;
	unsigned s;
	unsigned i;

	for (s = 0u; s < 3u; ++s) {
		const char *p = patterns[(1u == s) ? (unsigned)(letter - 'A') : 26u];

		for (i = 0u; i < 9u; ++i) {
			w[n++] = ('W' == p[i]) ? 10000u : 4000u;
		}
		if (s < 2u) {
			w[n++] = 4000u;
		}
	}
	memmove(w, &w[drop], (n - drop) * sizeof(w[0]));
	return n - drop;
}

static void add_analog(uint8_t type, uint8_t read_id, uint16_t value)
{
	uint8_t m[BAR_MSG_SIZE];

	bar_msg_pack(m, (bar_event_t)type, read_id, value);
	add(EV_MSG, m, BAR_MSG_SIZE);
}

/* LETTER: decoded on core 1; barcode_task looks it up and stores it. */
static void analog_letter_stored(void)
{
	reset();
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_analog(BAR_MSG_LETTER, 7u, bc_link_letter_value('B', true));
	add_mark(0);
	add_analog(BAR_MSG_LETTER, 8u, bc_link_letter_value('Q', false));	/* F in the table */
	add_mark(1);
	run();
	CHECK(NULL != strstr(g_log, "[barcode] 'B' -> RIGHT (dir=rev), stored for the next junction\n"));
	CHECK(cmd_is(g_mark_cmd[0], 'B', BARCODE_ACT_RIGHT));
	CHECK(cmd_is(g_mark_cmd[1], 'Q', BARCODE_ACT_FORWARD));
	CHECK(barcode_judged_word() == barcode_judged_pack(8u));
	CHECK((2 == g_e_n) && (1u == g_e_letter[0]) && (BARCODE_ACT_RIGHT == g_e_action[0]));
}

/* ANALOG: a read one bar short (27 elements: no stop END) is decoded on
   core 0 by Boon's repair; command, judged word, then the reply. */
static void analog_decoded_on_core0(void)
{
	uint32_t w[32];
	unsigned n = bc_letter('A', 2u, w);

	reset();
	bc_feed(w, n);
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_analog(BAR_MSG_ANALOG, g_bc.read_id, 0u);
	add_mark(0);
	run();
	CHECK(NULL != strstr(g_log, "[barcode] 'A' -> LEFT (dir=fwd), stored for the next junction\n"));
	CHECK(NULL != strstr(g_log, "[barcode]   analog read, pass 0 (swing), 27 elements"));
	CHECK(cmd_is(g_mark_cmd[0], 'A', BARCODE_ACT_LEFT));
	CHECK(barcode_judged_word() == barcode_judged_pack(g_bc.read_id));
	CHECK(bc_link_reply_is(barcode_analog_reply_word(), g_bc.read_id));
	CHECK(BC_VALID == bc_link_reply_result(barcode_analog_reply_word()));
	CHECK('A' == bc_link_reply_letter(barcode_analog_reply_word()));
}

/* A reject prints Boon's reason and the widths in mm, sends letter 31,
   stores the judged word and no command. */
static void analog_rejected(void)
{
	uint32_t w[32];
	unsigned n = bc_letter('C', 0u, w);

	n = 15u;			/* half a read */
	reset();
	bc_feed(w, n);
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_analog(BAR_MSG_ANALOG, g_bc.read_id, 0u);
	add_mark(0);
	run();
	CHECK(NULL != strstr(g_log, "[barcode] REJECTED: malformed (analog count, decode 0 us)\n"));
	CHECK(NULL != strstr(g_log, "[barcode]   15 widths in mm (1 letter = 29)"));
	CHECK(NULL != strstr(g_log, "[barcode]    0: B4.0 w10.0 B4.0 w4.0 B10.0"));
	/* ours: the partial decoder's verdict, before the widths */
	CHECK(log_before("[barcode]   partial: not tried (no reading of 25..35 elements, odd)\n",
			   "[barcode]   15 widths in mm"));
	CHECK(BARCODE_LETTER_NONE == barcode_cmd_letter(g_mark_cmd[0]));
	CHECK(barcode_judged_word() == barcode_judged_pack(g_bc.read_id));
	CHECK(BC_REJECT == bc_link_reply_result(barcode_analog_reply_word()));
	CHECK((1 == g_e_n) && (COMM_W_BARCODE_UNREADABLE == g_e_letter[0]));
}

/* Ours: the car read of 20 Nov 17:44 (M), a 27 that Boon's decoders
   reject, is *F* backwards with one thin bar lost: the partial decoder
   reads it (pass 7) and the console says so. */
static void analog_partial_decoded(void)
{
	static const uint32_t p1[27] = {
		1800u, 10900u, 11800u, 11700u, 11300u, 34800u, 1700u, 9600u, 2600u,
		8000u, 3100u, 20500u, 12900u, 11700u, 12300u, 10500u, 2300u, 9400u,
		2400u, 9000u, 14500u, 8500u, 14700u, 9100u, 2500u, 21300u, 1500u
	};

	reset();
	bc_feed(p1, 27u);
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_analog(BAR_MSG_ANALOG, g_bc.read_id, 0u);
	add_mark(0);
	run();
	CHECK(NULL != strstr(g_log, "[barcode] 'F' -> FORWARD (dir=rev), stored for the next junction\n"));
	CHECK(NULL != strstr(g_log, "[barcode]   analog read, pass 7 (partial: 1 missed bar), 27 elements"));
	CHECK(cmd_is(g_mark_cmd[0], 'F', BARCODE_ACT_FORWARD));
	CHECK('F' == bc_link_reply_letter(barcode_analog_reply_word()));
}

/* Ours: *A* with its first data space lost reads exactly as *B* with the
   space after its first data bar lost (wide + narrow + narrow either
   way): the partial decoder refuses, and says why. */
static void analog_partial_refused(void)
{
	uint32_t w[32];
	unsigned n = bc_letter('A', 0u, w);

	w[10] += w[11] + w[12];
	memmove(&w[11], &w[13], (n - 13u) * sizeof(w[0]));
	n -= 2u;
	reset();
	bc_feed(w, n);
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_analog(BAR_MSG_ANALOG, g_bc.read_id, 0u);
	add_mark(0);
	run();
	CHECK(log_before("[barcode] REJECTED: malformed (analog ", "[barcode]   partial: best"));
	CHECK(NULL != strstr(g_log, "[barcode]   partial: best 'A' (fwd, 1 missed space), refused: 'B' fits nearly as well (x1.0)\n"));
	CHECK(BARCODE_LETTER_NONE == barcode_cmd_letter(g_mark_cmd[0]));
	CHECK(BC_REJECT == bc_link_reply_result(barcode_analog_reply_word()));
}

/* Cut by a junction: judged for the console only, nothing stored. */
static void analog_cut(void)
{
	uint32_t w[32];
	unsigned n = bc_letter('D', 2u, w);

	reset();
	bc_feed(w, n);
	add(EV_WAIT, NULL, 0);
	add_mission(MS_RUNNING);
	add_analog(BAR_MSG_ANALOG, g_bc.read_id, (uint16_t)BAR_ANALOG_CUT);
	add_mark(0);
	run();
	CHECK(NULL != strstr(g_log, "[barcode] 'D' read, but a junction cut the read: not stored"));
	CHECK(BARCODE_LETTER_NONE == barcode_cmd_letter(g_mark_cmd[0]));
	CHECK(bc_link_reply_is(barcode_analog_reply_word(), g_bc.read_id));
}
#endif

int main(void)
{
#if BARCODE_SENSOR_ANALOG
	analog_letter_stored();
	analog_decoded_on_core0();
	analog_rejected();
	analog_partial_decoded();
	analog_partial_refused();
	analog_cut();
#endif
#if !BARCODE_DEFAULT_TABLE_SET
	no_table();
#elif defined(TEST_BAD_TABLE)
	bad_table();
#else
	compiled_table();
#endif
	mqtt_map_every_action();
	map_during_a_read();
	map_between_read_and_junction();
	map_wrong_length();
	map_survives_new_mission();
#if !BARCODE_DEFAULT_TABLE_SET
	maps_queue_full();
	return check_report("barcode_task (no compiled-in table)");
#elif defined(TEST_BAD_TABLE)
	maps_queue_full();
	return check_report("barcode_task (compiled-in table with a bad character)");
#elif BARCODE_SENSOR_ANALOG
	return check_report("barcode_task (analog path, compiled-in " BARCODE_DEFAULT_TABLE ")");
#else
	return check_report("barcode_task (compiled-in " BARCODE_DEFAULT_TABLE ")");
#endif
}
