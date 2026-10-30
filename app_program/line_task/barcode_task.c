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
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../common/messages.h"
#include "../comms_task/comms_task.h"
#include "../nav_task/nav_ctrl.h"
#include "barcode_task.h"
#include "barcode_rx.h"

#define BAR_MBF_BYTES		512	/* ~64 queued 4-byte messages        */
#define MAP_MBF_BYTES		32	/* two queued 7-byte maps            */
#define BARCODE_WAIT_MS		20	/* also how often mission is checked */
#define DUMP_MAX		64u	/* widths printed after a reject     */
#define DUMP_PER_LINE		10u

LOCAL ID		g_bar_mbfid;
LOCAL ID		g_map_mbfid;
LOCAL volatile uint32_t	g_cmd = BARCODE_CMD_NONE;	/* one store, this task only */
LOCAL volatile UB	g_map_version;
LOCAL volatile BOOL	g_map_loaded;
LOCAL uint32_t		g_cmd_seq;
LOCAL barcode_rx_t	g_rx;		/* ~1.4 KB: static, not on the stack */
LOCAL barcode_table_t	g_table;

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

LOCAL void report(const barcode_verdict_t *v)
{
	UB	body[COMM_W_INTAKE_SIZE_BARCODE];
	UB	line[112];

	if(v->accepted) {
		set_cmd(barcode_letter_index(v->letter), (uint32_t)v->action);
		tm_printf((UB *)"[barcode] '%c' -> %s (dir=%s), stored for the next junction\n",
			  v->letter, barcode_action_name((uint32_t)v->action),
			  v->decode.reversed ? "rev" : "fwd");
		(void)comms_tele_send(body, (INT)comm_w_pack_barcode(body,
			barcode_letter_index(v->letter), (uint32_t)v->action));
		return;
	}

	/* rejected: no command; the stored one (if any) is kept */
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
	   (BARCODE_REJECT_NOT_ONE_LETTER == v->reject)) {
		dump_widths();
	}
	(void)comms_tele_send(body, (INT)comm_w_pack_barcode(body,
		COMM_W_BARCODE_UNREADABLE, 0U));
}

/* Only between barcodes, so one read always sees one table. */
LOCAL void apply_maps(void)
{
	UB	map[BARCODE_MAP_SIZE];
	INT	size;

	size = tk_rcv_mbf(g_map_mbfid, map, TMO_POL);
	while(0 < size) {
		if(barcode_table_apply_map(&g_table, map, size)) {
			g_map_version = g_table.version;
			g_map_loaded = TRUE;
			tm_printf((UB *)"[barcode] map v%u applied\n", (UINT)g_table.version);
		} else {
			tm_printf((UB *)"[barcode] map of %d bytes rejected: table kept\n", size);
		}
		size = tk_rcv_mbf(g_map_mbfid, map, TMO_POL);
	}
}

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

	(void)stacd;
	(void)exinf;

	barcode_rx_init(&g_rx);
	barcode_table_init(&g_table);
	g_map_loaded = g_table.loaded ? TRUE : FALSE;
	if(E_OK != barcode_task_init()) {
		tm_printf((UB *)"[barcode] bar_mbf/map_mbf not created: stopped\n");
		while(1) {
			(void)tk_slp_tsk(TMO_FEVR);
		}
	}
	tm_printf((UB *)"[barcode] decoder ready, table: %s\n",
		  g_table.loaded ? "compiled-in default" : "none until a map arrives");

	while(1) {
		size = tk_rcv_mbf(g_bar_mbfid, msg, BARCODE_WAIT_MS);
		if(BAR_MSG_SIZE == size) {
			event = barcode_rx_feed(&g_rx, msg);
			if(BARCODE_RX_NONE != event) {
				barcode_rx_evaluate(&g_rx, event, &g_table, &verdict);
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
