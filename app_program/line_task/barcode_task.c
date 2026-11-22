/*
 * barcode_task.c - decode buffered barcode reads, apart from sampling.
 *
 * ir_sense_task never waits for anything here: it sends widths with
 * TMO_POL and aborts the read if bar_mbf is full.  Decoding, the table
 * lookup, console output and telemetry all happen on this side.
 *
 * Mission changes: a read in progress is dropped on any change (ir_sense_task
 * also aborts it); the stored command survives a pause, and is cleared on a
 * new mission, Finished and Fault.
 *
 * BARCODE_SENSOR_ANALOG 1: the analog path's LETTER and ANALOG messages
 * (bc_analog_link.h) are judged here too.  For ANALOG this task runs
 * Boon's end-of-code decode (bc_finish(), with our partial-read decoder
 * last) on ir_sense_task's capture, stores the command and the judged
 * word, then the reply word, and only then prints: the pass that decoded
 * (and for the partial decoder the elements it found lost), or after a
 * REJECTED what the partial decoder made of the read and why it refused.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../common/messages.h"
#include "../comms_task/comms_task.h"
#include "../nav_task/nav_ctrl.h"
#include "barcode_task.h"
#include "barcode_rx.h"
#if BARCODE_SENSOR_ANALOG
#include "bc_analog_link.h"
#endif

#define BAR_MBF_BYTES		512	/* ~64 queued 4-byte messages        */
#define MAP_MBF_BYTES		32	/* two queued 7-byte maps            */
#define BARCODE_WAIT_MS		20	/* also how often mission is checked */
#define DUMP_MAX		64u	/* widths printed after a reject     */
#define DUMP_PER_LINE		10u

LOCAL ID		g_bar_mbfid;
LOCAL ID		g_map_mbfid;
LOCAL volatile uint32_t	g_cmd = BARCODE_CMD_NONE;	/* one store, this task only */
LOCAL volatile uint32_t	g_judged;	/* barcode_rx.h: after g_cmd, one store */
LOCAL volatile UB	g_map_version;
LOCAL volatile BOOL	g_map_loaded;
LOCAL uint32_t		g_cmd_seq;
LOCAL barcode_rx_t	g_rx;		/* ~1.4 KB: static, not on the stack */
LOCAL barcode_table_t	g_table;
#if BARCODE_SENSOR_ANALOG
/* microseconds (RP2040 TIMERAWL, 1 MHz), to print how long a decode took */
#if defined(__arm__) && defined(TIMER_BASE)
#define ANALOG_US()	((uint32_t)in_w(TIMER_BASE + 0x28u))
#else
#define ANALOG_US()	(0u)		/* host test */
#endif
LOCAL volatile uint32_t	g_analog_reply;	/* bc_analog_link.h: one store, this task only */
LOCAL uint32_t		g_aw[BC_MAX_ELEMENTS + 1u];	/* a rejected read's widths, um */
#endif

EXPORT ER barcode_task_init(void)
{
	T_CMBF	cmbf = { .exinf = NULL, .mbfatr = TA_TFIFO,
			 .bufsz = BAR_MBF_BYTES, .maxmsz = BAR_MSG_SIZE };
	ID	objid;

	if(0 < g_bar_mbfid) {
		return E_OK;
	}
	objid = tk_cre_mbf(&cmbf);
	if(0 >= objid) {
		return (ER)objid;
	}
	g_bar_mbfid = objid;

	cmbf.bufsz = MAP_MBF_BYTES;
	cmbf.maxmsz = BARCODE_MAP_SIZE;
	objid = tk_cre_mbf(&cmbf);
	if(0 >= objid) {
		return (ER)objid;
	}
	g_map_mbfid = objid;
	return E_OK;
}

EXPORT ID barcode_bar_mbf(void)
{
	return g_bar_mbfid;
}

EXPORT ER barcode_map_submit(const UB *map, INT length)
{
	if((NULL == map) || (BARCODE_MAP_SIZE != length)) {
		return E_PAR;
	}
	if(0 >= g_map_mbfid) {
		return E_OBJ;
	}
	return tk_snd_mbf(g_map_mbfid, (void *)map, length, TMO_POL);
}

EXPORT uint32_t barcode_cmd_word(void)
{
	return g_cmd;
}

EXPORT uint32_t barcode_judged_word(void)
{
	return g_judged;
}

EXPORT UB barcode_map_version(void)
{
	return g_map_version;
}

EXPORT BOOL barcode_map_loaded(void)
{
	return g_map_loaded;
}

LOCAL void set_cmd(uint32_t letter_index, uint32_t action)
{
	g_cmd_seq++;
	g_cmd = barcode_cmd_pack(letter_index, action, g_cmd_seq);	/* one store */
}

/* The rejected read's widths, in ms (1 tick = 1 ms): B bar, w space. */
LOCAL void dump_widths(void)
{
	UB	line[128];
	UB	*p = line;
	uint16_t	shown = (g_rx.count > DUMP_MAX) ? (uint16_t)DUMP_MAX : g_rx.count;
	uint16_t	i;

	tm_printf((UB *)"[barcode]   %u widths (1 letter = 29)\n", (UINT)g_rx.count);
	for(i = 0u; i < shown; ++i) {
		if(0u == (i % DUMP_PER_LINE)) {
			p = line;
			p += tm_sprintf(p, (UB *)"[barcode]   %2u:", (UINT)i);
		}
		p += tm_sprintf(p, (UB *)" %c%u", (0u == (i & 1u)) ? 'B' : 'w',
				(UINT)g_rx.widths[i]);
		if((0u == ((i + 1u) % DUMP_PER_LINE)) || ((i + 1u) == shown)) {
			*p = '\0';
			tm_printf((UB *)"%s\n", line);		/* one call per line */
		}
	}
}

/* Console and telemetry, after the verdict is stored (store_verdict). */
/* Failed reads worth a second try (21 Nov 2026, user: re-read a barcode
   once, slowly and straight, if the first read fails; motion_task backs up
   and reads it again, MOTION_BC_RETRY).  A count, one aligned word. */
LOCAL volatile uint32_t	g_retry_req;

EXPORT uint32_t barcode_retry_req(void)
{
	return g_retry_req;
}

LOCAL void report(const barcode_verdict_t *v)
{
	UB	body[COMM_W_INTAKE_SIZE_BARCODE];
	UB	line[112];

	if(v->accepted) {
		tm_printf((UB *)"[barcode] '%c' -> %s (dir=%s), stored for the next junction\n",
			  v->letter, barcode_action_name((uint32_t)v->action),
			  v->decode.reversed ? "rev" : "fwd");
		(void)comms_tele_send(body, (INT)comm_w_pack_barcode(body,
			barcode_letter_index(v->letter), (uint32_t)v->action));
		return;
	}

	/* rejected: no command; the stored one (if any) is kept */
	if((BARCODE_REJECT_ABORTED == v->reject) &&
	   (BAR_ABORT_JUNCTION == g_rx.abort_reason)) {
		return;		/* a crossbar, not a barcode: nothing to say */
	}
	if(BARCODE_REJECT_ABORTED == v->reject) {
		(void)tm_sprintf(line, (UB *)"[barcode] REJECTED: aborted (%s)\n",
				 bar_abort_name(g_rx.abort_reason));
	} else if(BARCODE_REJECT_DECODE == v->reject) {
		(void)tm_sprintf(line, (UB *)"[barcode] REJECTED: malformed (%s)\n",
				 barcode_decode_error_name(v->decode.error));
	} else if(BARCODE_REJECT_UNKNOWN == v->reject) {
		(void)tm_sprintf(line, (UB *)"[barcode] REJECTED: '%c' unknown (%s)\n",
				 v->letter, g_table.loaded ? "not in table"
							   : "no barcode map loaded");
	} else {
		(void)tm_sprintf(line, (UB *)"[barcode] REJECTED: %s\n",
				 barcode_reject_name(v->reject));
	}
	tm_printf((UB *)"%s", line);
	if((BARCODE_REJECT_DECODE == v->reject) ||
	   (BARCODE_REJECT_NOT_ONE_LETTER == v->reject) ||
	   ((BARCODE_REJECT_ABORTED == v->reject) &&
	    (BAR_ABORT_CANCELLED != g_rx.abort_reason))) {
		g_retry_req++;		/* a garbled or cut-off read */
	}
	if((BARCODE_REJECT_DECODE == v->reject) ||
	   (BARCODE_REJECT_NOT_ONE_LETTER == v->reject)) {
		dump_widths();
	}
	(void)comms_tele_send(body, (INT)comm_w_pack_barcode(body,
		COMM_W_BARCODE_UNREADABLE, 0U));
}

/* The command first, then the judged word: ir_sense_task arms the
   junction only once it sees this read judged and a new command stored,
   so nav always finds the command at the junction.  Both before the
   console, which can take milliseconds. */
LOCAL void store_verdict(const barcode_verdict_t *v)
{
	if(v->accepted) {
		set_cmd(barcode_letter_index(v->letter), (uint32_t)v->action);
	}
	g_judged = barcode_judged_pack(g_rx.read_id);		/* one store */
}

#if BARCODE_SENSOR_ANALOG
/* GAMBLE (bc_analog_partial.h): the candidates are the letters with a
   turn (not straight on), A first; none: every letter. */
LOCAL void gamble_candidates(void)
{
	char	c[27];
	UINT	n = 0U;
	UINT	i;

	if(g_table.loaded) {
		for(i = 0U; i < 26U; i++) {
			if((uint8_t)BARCODE_ACT_FORWARD != g_table.action[i]) {
				c[n++] = (char)('A' + i);
			}
		}
	}
	c[n] = '\0';
	bc_partial_set_candidates(c);
	if(0U != BC_PART_GAMBLE) {
		tm_printf((UB *)"[barcode] gamble candidates: %s\n", (0U != n) ? c : "every letter");
	}
}
#endif

/* Only between barcodes, so one read always sees one table. */
LOCAL void apply_maps(void)
{
	UB	map[BARCODE_MAP_SIZE];
	char	text[BARCODE_TABLE_TEXT_SIZE];
	INT	size;

	size = tk_rcv_mbf(g_map_mbfid, map, TMO_POL);
	while(0 < size) {
		if(barcode_table_apply_map(&g_table, map, size)) {
#if BARCODE_SENSOR_ANALOG
			gamble_candidates();
#endif
			g_map_version = g_table.version;
			g_map_loaded = TRUE;
			barcode_table_to_string(&g_table, text);
			tm_printf((UB *)"[barcode] map v%u applied: %s\n",
				  (UINT)g_table.version, text);
		} else {
			tm_printf((UB *)"[barcode] map of %d bytes rejected: table kept\n", size);
		}
		size = tk_rcv_mbf(g_map_mbfid, map, TMO_POL);
	}
}

#if BARCODE_SENSOR_ANALOG
EXPORT uint32_t barcode_analog_reply_word(void)
{
	return g_analog_reply;
}

/* Boon's reason codes (bc_analog_code39.h) in REJECTED lines. */
LOCAL const char *analog_reason_name(uint8_t reason)
{
	switch((code39_result_t)reason) {
	case CODE39_NOT_BARCODE:
		return "not_barcode";
	case CODE39_ERR_COUNT:
		return "count";
	case CODE39_ERR_RATIO:
		return "ratio";
	case CODE39_ERR_PATTERN:
		return "pattern";
	case CODE39_ERR_START_STOP:
		return "start_stop";
	case CODE39_ERR_LENGTH:
		return "length";
	case CODE39_ERR_TOO_LONG:
		return "too_long";
	case CODE39_ERR_TOO_MANY:
		return "too_many";
	case CODE39_OK:
	default:
		return "unknown";
	}
}

/* How an analog read was decoded (bc_out_t.pass), for the console. */
LOCAL const char *analog_pass_name(uint8_t pass)
{
	static const char *const names[8] = {
		"swing", "mid", "low", "adaptive", "best match",
		"end speck dropped", "best match, end speck dropped",
		(0U != BC_PART_GAMBLE) ? "GAMBLE" : "partial"
	};

	return (pass < 8u) ? names[pass] : "?";
}

/* One count of the partial decoder's defects: ", 2 missed bars". */
LOCAL UB *partial_count(UB *p, bool *first, uint8_t n, const char *what)
{
	if(0u != n) {
		p += tm_sprintf(p, (UB *)"%s%u %s%s", *first ? "" : ", ", (UINT)n, what,
				(1u == n) ? "" : "s");
		*first = false;
	}
	return p;
}

/* What the partial decoder lined up: "1 missed bar, 1 speck". */
LOCAL UB *partial_how(UB *p, const bc_part_t *pt)
{
	bool	first = true;

	p = partial_count(p, &first, pt->missed_bars, "missed bar");
	p = partial_count(p, &first, pt->missed_spaces, "missed space");
	p = partial_count(p, &first, pt->specks, "speck");
	p = partial_count(p, &first, (uint8_t)(pt->clip_front + pt->clip_back),
			  "clipped element");
	if(first) {
		p += tm_sprintf(p, (UB *)"no element lost");
	}
	return p;
}

/* After an analog REJECTED: what the partial decoder made of the read
   and why it gave no letter (nothing if it did not run). */
LOCAL void partial_report(const bc_part_t *pt)
{
	UB	line[160];
	UB	*p = line;
	uint32_t	x10;

	if((uint8_t)BC_PART_NOT_RUN == pt->reason) {
		return;
	}
	p += tm_sprintf(p, (UB *)"[barcode]   partial: ");
	if((uint8_t)BC_PART_COUNT == pt->reason) {
		p += tm_sprintf(p, (UB *)"not tried (no reading of %u..%u elements, odd)",
				(UINT)BC_PART_N_MIN, (UINT)BC_PART_N_MAX);
	} else if((uint8_t)BC_PART_LEVELS == pt->reason) {
		p += tm_sprintf(p, (UB *)"refused, narrow and wide widths not a code's");
	} else if((uint8_t)BC_PART_BUSY == pt->reason) {
		p += tm_sprintf(p, (UB *)"refused, search limit reached");
	} else {
		p += tm_sprintf(p, (UB *)"best '%c' (%s, ", pt->ch,
				pt->b_reversed ? "rev" : "fwd");
		p = partial_how(p, pt);
		p += tm_sprintf(p, (UB *)"), refused: ");
		x10 = (0u != pt->cost) ? (uint32_t)(((uint64_t)pt->cost2 * 10u) / pt->cost) : 0u;
		if((uint8_t)BC_PART_MARGIN == pt->reason) {
			p += tm_sprintf(p, (UB *)"'%c' fits nearly as well (x%u.%u)",
					(0 != pt->ch2) ? pt->ch2 : '?',
					(UINT)(x10 / 10u), (UINT)(x10 % 10u));
		} else if((uint8_t)BC_PART_STAR == pt->reason) {
			p += tm_sprintf(p, (UB *)"no '*' seen whole");
		} else if((uint8_t)BC_PART_EDITS == pt->reason) {
			p += tm_sprintf(p, (UB *)"more than %u elements lost or split",
					(UINT)BC_PART_K);
		} else {
			p += tm_sprintf(p, (UB *)"fit too poor");
		}
	}
	*p = '\0';
	tm_printf((UB *)"%s\n", line);
}

/* A read's first (swing) widths, in mm to 0.1 mm: B bar, w space. */
LOCAL void analog_dump(uint16_t n, const uint16_t counts[BC_READINGS])
{
	UB	line[128];
	UB	*p = line;
	uint16_t	i;

	tm_printf((UB *)"[barcode]   %u widths in mm (1 letter = 29); readings swing/mid/low/adapt20/15/10: %u/%u/%u/%u/%u/%u\n",
		  (UINT)n, (UINT)counts[0], (UINT)counts[1], (UINT)counts[2],
		  (UINT)counts[3], (UINT)counts[4], (UINT)counts[5]);
	for(i = 0u; i < n; ++i) {
		if(0u == (i % DUMP_PER_LINE)) {
			p = line;
			p += tm_sprintf(p, (UB *)"[barcode]   %2u:", (UINT)i);
		}
		p += tm_sprintf(p, (UB *)" %c%u.%u", (0u == (i & 1u)) ? 'B' : 'w',
				(UINT)(g_aw[i] / 1000u), (UINT)((g_aw[i] % 1000u) / 100u));
		if((0u == ((i + 1u) % DUMP_PER_LINE)) || ((i + 1u) == n)) {
			*p = '\0';
			tm_printf((UB *)"%s\n", line);		/* one call per line */
		}
	}
}

/* A letter from the analog path: the table, the command and the judged
   word (as store_verdict), then the console and telemetry (as report). */
LOCAL void analog_judge(uint8_t read_id, char letter, bool reversed, uint8_t pass)
{
	UB		body[COMM_W_INTAKE_SIZE_BARCODE];
	barcode_action_t	action = BARCODE_ACT_FORWARD;
	bool		known = barcode_table_lookup(&g_table, letter, &action);

	if(known) {
		set_cmd(barcode_letter_index(letter), (uint32_t)action);
	}
	g_judged = barcode_judged_pack(read_id);		/* one store */
	if(known) {
		tm_printf((UB *)"[barcode] '%c' -> %s (dir=%s), stored for the next junction\n",
			  letter, barcode_action_name((uint32_t)action),
			  reversed ? "rev" : "fwd");
		tm_printf((UB *)"[barcode]   analog read, pass %u (%s)\n", (UINT)pass,
			  analog_pass_name(pass));
		(void)comms_tele_send(body, (INT)comm_w_pack_barcode(body,
			barcode_letter_index(letter), (uint32_t)action));
		return;
	}
	tm_printf((UB *)"[barcode] REJECTED: '%c' unknown (%s)\n", letter,
		  g_table.loaded ? "not in table" : "no barcode map loaded");
	(void)comms_tele_send(body, (INT)comm_w_pack_barcode(body,
		COMM_W_BARCODE_UNREADABLE, 0U));
}

/* ANALOG: decode ir_sense_task's capture (it waits in BC_ST_DECODING),
   store, copy what the console needs, reply, then print. */
LOCAL void analog_decode(uint8_t read_id, bool cut)
{
	UB		body[COMM_W_INTAKE_SIZE_BARCODE];
	const bc_capture_t	*cap = line_bc_capture();
	bc_out_t	o;
	bc_result_t	r;
	uint16_t	counts[BC_READINGS];
	uint16_t	n = 0u;
	uint8_t		k;
	uint32_t	t0;
	uint32_t	us;

	BC_LINK_BARRIER();	/* the capture as core 1 left it */
	t0 = ANALOG_US();
	r = bc_finish(cap, &o);
	us = ANALOG_US() - t0;
	if((BC_REJECT == r) || ((BC_VALID != r) && cut)) {
		for(k = (uint8_t)(BC_READINGS - 1u); k > 0u; k--) {
			counts[k] = bc_reading(cap, k, g_aw);
		}
		n = bc_reading(cap, 0u, g_aw);
		counts[0] = n;
	}
	if((BC_VALID == r) && !cut) {
		/* the command and the judged word before the reply */
		barcode_action_t	action = BARCODE_ACT_FORWARD;

		if(barcode_table_lookup(&g_table, o.ch, &action)) {
			set_cmd(barcode_letter_index(o.ch), (uint32_t)action);
		}
		g_judged = barcode_judged_pack(read_id);
	} else if(BC_REJECT == r) {
		g_judged = barcode_judged_pack(read_id);
	} else {
		/* not a barcode, or cut by a junction: nothing to arm */
	}
	BC_LINK_BARRIER();	/* done with the capture before it goes back */
	g_analog_reply = bc_link_reply_pack(read_id, r, o.ch);	/* one store */

	if((BC_VALID == r) && !cut) {
		barcode_action_t	action = BARCODE_ACT_FORWARD;

		if(barcode_table_lookup(&g_table, o.ch, &action)) {
			UB	how[96];

			tm_printf((UB *)"[barcode] '%c' -> %s (dir=%s), stored for the next junction\n",
				  o.ch, barcode_action_name((uint32_t)action),
				  o.b_reversed ? "rev" : "fwd");
			(void)tm_sprintf(how, (UB *)"%s", analog_pass_name(o.pass));
			if(7u == o.pass) {
				UB	*p = how + tm_sprintf(how, (UB *)"partial: ");

				p = partial_how(p, &o.part);
				*p = '\0';
			}
			tm_printf((UB *)"[barcode]   analog read, pass %u (%s), %u elements, %u mm, decoded in %u us\n",
				  (UINT)o.pass, how, (UINT)o.elements, (UINT)(o.length_um / 1000u),
				  (UINT)us);
			(void)comms_tele_send(body, (INT)comm_w_pack_barcode(body,
				barcode_letter_index(o.ch), (uint32_t)action));
			return;
		}
		tm_printf((UB *)"[barcode] REJECTED: '%c' unknown (%s)\n", o.ch,
			  g_table.loaded ? "not in table" : "no barcode map loaded");
	} else if(BC_VALID == r) {
		tm_printf((UB *)"[barcode] '%c' read, but a junction cut the read: not stored (analog, pass %u, %s)\n",
			  o.ch, (UINT)o.pass, analog_pass_name(o.pass));
	} else if(BC_REJECT == r) {
		tm_printf((UB *)"[barcode] REJECTED: malformed (analog %s, decode %u us)\n",
			  analog_reason_name(o.reason), (UINT)us);
		partial_report(&o.part);
		analog_dump(n, counts);
	} else if(cut) {
		tm_printf((UB *)"[barcode] REJECTED: a junction cut the read (analog %s)\n",
			  analog_reason_name(o.reason));
		partial_report(&o.part);
		analog_dump(n, counts);
	} else {
		return;		/* a crossing line or a speck: nothing to say */
	}
	(void)comms_tele_send(body, (INT)comm_w_pack_barcode(body,
		COMM_W_BARCODE_UNREADABLE, 0U));
}

/* The analog path's own messages; false for the others (ABORT goes to
   barcode_rx.c as for GP7). */
LOCAL BOOL analog_feed(const UB *msg)
{
	uint16_t	value = (uint16_t)(msg[2] | ((uint16_t)msg[3] << 8));

	if(BAR_MSG_LETTER == msg[0]) {
		analog_judge(msg[1], (char)(value & 0xFFu), (value & 0x100u) != 0u, 0u);
		return TRUE;
	}
	if(BAR_MSG_ANALOG == msg[0]) {
		analog_decode(msg[1], (value & BAR_ANALOG_CUT) != 0u);
		return TRUE;
	}
	return FALSE;
}
#endif

LOCAL void on_mission_change(mission_state_t prev, mission_state_t next)
{
	bool	keep;

	barcode_rx_discard(&g_rx);		/* partial read: never carried over */
	keep = ((MS_RUNNING == next) && (MS_PAUSED == prev)) ||	/* resume */
	       ((MS_PAUSED == next) && (MS_RUNNING == prev));	/* pause  */
	if(!keep && (BARCODE_LETTER_NONE != barcode_cmd_letter(g_cmd))) {
		set_cmd(BARCODE_LETTER_NONE, 0U);
		tm_printf((UB *)"[barcode] mission %s: stored command cleared\n",
			  nav_state_name(next));
	}
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	UB		msg[BAR_MSG_SIZE];
	INT		size;
	barcode_rx_event_t	event;
	barcode_verdict_t	verdict;
	mission_state_t	mission = nav_mission_state();
	mission_state_t	now;
	char		text[BARCODE_TABLE_TEXT_SIZE];

	(void)stacd;
	(void)exinf;

	barcode_rx_init(&g_rx);
	barcode_table_init(&g_table);
#if BARCODE_SENSOR_ANALOG
	gamble_candidates();
#endif
	g_map_loaded = g_table.loaded ? TRUE : FALSE;
	if(E_OK != barcode_task_init()) {
		tm_printf((UB *)"[barcode] bar_mbf/map_mbf not created: stopped\n");
		while(1) {
			(void)tk_slp_tsk(TMO_FEVR);
		}
	}
	/* The table the car starts with, so the image can be checked on the
	   console before a run: a compiled-in one is printed letter by letter,
	   A first; one that failed to parse is named, not silently dropped. */
	barcode_table_to_string(&g_table, text);
	if(g_table.loaded) {
		tm_printf((UB *)"[barcode] decoder ready, table: compiled-in default %s\n",
			  text);
	} else if(0 != BARCODE_DEFAULT_TABLE_SET) {
		tm_printf((UB *)"[barcode] decoder ready, table: none until a map arrives "
			  "(compiled-in table is not 26 of F/L/R/U)\n");
	} else {
		tm_printf((UB *)"[barcode] decoder ready, table: none until a map arrives\n");
	}

	while(1) {
		size = tk_rcv_mbf(g_bar_mbfid, msg, BARCODE_WAIT_MS);
#if BARCODE_SENSOR_ANALOG
		if((BAR_MSG_SIZE == size) && analog_feed(msg)) {
			size = 0;	/* judged above */
		}
#endif
		if(BAR_MSG_SIZE == size) {
			event = barcode_rx_feed(&g_rx, msg);
			if(BARCODE_RX_NONE != event) {
				barcode_rx_evaluate(&g_rx, event, &g_table, &verdict);
				store_verdict(&verdict);
				report(&verdict);
			}
		}

		now = nav_mission_state();
		if(now != mission) {
			on_mission_change(mission, now);
			mission = now;
		}
		if(barcode_rx_idle(&g_rx)) {
			apply_maps();
		}
	}
}
