/*
 *	comms_task.c
 *	Buddy 1: car state -> MQTT records (docs/comms/README.md).
 *
 *	Intake: the other tasks pack an event body with their comm_w_pack_*
 *	helper (common/messages.h) and hand it over with comms_tele_send(), which
 *	posts to tele_mbf and never blocks.  tele_mbf and comms_tele_send() exist
 *	in every build, so producers need no build conditionals.
 *
 *	comms_task (MQTT builds only) waits on tele_mbf between heartbeats.  Each
 *	body is length-checked for its kind; a malformed one gets one console line
 *	and changes nothing.  A valid one updates the S cache and is published as
 *	record E: the header is stamped with s and up, and the body is copied
 *	unchanged (MQTT_SCHEMA.md section 5).  Once a second it publishes H.  It
 *	never touches lwIP: records go to the radio task through
 *	cyw43_utk_mqtt_send(), and a full queue drops the record and says so.
 *
 *	Priority 16, one below nav_ctrl_task (Week 6 report), pinned to
 *	processor 1 (RP2040 core 0) with the radio, away from the control tasks
 *	on core 1.  The core is proposed, still being agreed with Buddy 2.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "comms_task.h"
#include "../common/messages.h"

/* ------------------------------------------------------------------ *
 *  Intake queue -- every build
 * ------------------------------------------------------------------ */

#define TELE_DEPTH	16	/* records (design section 3.2) */
/* The message buffer stores a 4-byte header plus the message rounded up to
   4 bytes (kernel/tkernel/messagebuf.h), so size it for the largest body. */
#define TELE_SLOT	(4 + ((COMM_W_INTAKE_SIZE_MAX + 3) & ~3))

LOCAL UW	tele_buf[(TELE_DEPTH * TELE_SLOT) / sizeof(UW)];
LOCAL volatile UW	tele_dropped;	/* sends refused: queue full */

EXPORT ID	tele_mbfid;

EXPORT ER comms_tele_init(void)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = sizeof tele_buf;
	cmbf.maxmsz = COMM_W_INTAKE_SIZE_MAX;
	cmbf.bufptr = tele_buf;
	tele_mbfid = tk_cre_mbf(&cmbf);
	return (tele_mbfid < E_OK) ? tele_mbfid : E_OK;
}

EXPORT ER comms_tele_send(const UB *body, INT length)
{
	ER	er;

	if(tele_mbfid <= 0) return E_NOEXS;
	er = tk_snd_mbf(tele_mbfid, (void *)body, length, TMO_POL);
	if(er < E_OK) tele_dropped++;
	return er;
}

#if TM_WIFI_MQTT
#include "cyw43_utk.h"

#define COMMS_PRIORITY	16
#define COMMS_STACK	4096
#define H_PERIOD_MS	1000	/* the S cadence (NFR-10); H rides on it */

/* Bench check with no producers: every 5 s inject one synthetic event of
   each kind in turn, plus one malformed body, through comms_tele_send(), so
   the whole intake -> E path runs on hardware.  Off by default. */
#ifndef TM_COMMS_E_SELFTEST
#define TM_COMMS_E_SELFTEST	0
#endif

/* A wire E must fit the radio's outbound record. */
typedef char comms_e_fits[(COMM_W_E_SIZE_LIMIT <= CYW43_MQTT_RECORD_MAX) ? 1 : -1];

/* The latest value per source, for record S.  Fields are stored as on the
   wire (MQTT_SCHEMA.md section 4).  Only comms_task touches it. */
LOCAL struct {
	UB	mot, dst, lin, dev, bar, act, hum, hgt, obs, byp, mis, drp;
	BOOL	have_dist;
	UH	dist_at_change;		/* dist_mm when mot last changed */
} s_cache = { .dev = 128, .bar = COMM_W_BARCODE_UNREADABLE };

LOCAL UW	malformed;

/* ms since boot.  The low word wraps after ~49.7 days (design section 4.1). */
LOCAL UW now_ms(void)
{
	SYSTIM	t;

	tk_get_otm(&t);
	return t.lo;
}

/* Fold one valid event into the S cache. */
LOCAL void fold(const T_COMM_W_BODY *b)
{
	UW	cm;

	switch(b->kind) {
	case COMM_W_KIND_MOTION:
		/* dst is cm since the last motion change, derived from the
		   wrapping dist_mm counter (handles one wrap), clamped to 8 bits. */
		if(!s_cache.have_dist || b->motion_state != s_cache.mot) {
			s_cache.dist_at_change = b->dist_mm;
			s_cache.have_dist = TRUE;
		}
		s_cache.mot = b->motion_state;
		cm = comm_w_dist_delta_mm(b->dist_mm, s_cache.dist_at_change) / 10U;
		s_cache.dst = (UB)((cm > 255U) ? 255U : cm);
		break;
	case COMM_W_KIND_LINE:
		s_cache.lin = b->line_state;
		s_cache.dev = b->deviation;
		break;
	case COMM_W_KIND_BARCODE:
		s_cache.bar = b->letter;
		s_cache.act = b->action;
		break;
	case COMM_W_KIND_IMU:
		s_cache.hum = b->hump_count;
		s_cache.hgt = b->hump_height_mm;
		break;
	case COMM_W_KIND_OBSTACLE:
		s_cache.obs = b->width_cm;
		s_cache.byp = b->bypass;
		break;
	default:	/* COMM_W_KIND_MISSION */
		s_cache.mis = b->mission_state;
		break;
	}
}

/* Print a record's bytes and fate as one console line. */
LOCAL void print_record(const char *what, UH seq, UW up, const UB *rec,
			UW len, ER er)
{
	UB	line[128];
	UB	*p = line;
	UW	i;

	p += tm_sprintf(p, (UB *)"[comms] %s s=%u up=%u bytes=", what, seq, up);
	for(i = 0; i < len; i++) {
		p += tm_sprintf(p, (UB *)"%02x", rec[i]);
	}
	if(er == E_OK) {
		tm_sprintf(p, (UB *)" queued\n");
	} else {
		tm_sprintf(p, (UB *)" dropped(%d)\n", er);
	}
	tm_printf(line);
}

/* One intake message: check, fold, publish as E. */
LOCAL void handle_intake(const UB *body, INT size, UH *seq)
{
	static const char *const kind_name[COMM_W_KIND_COUNT] = {
		"E motion", "E line", "E barcode", "E imu", "E obstacle", "E mission"
	};
	T_COMM_W_BODY	b = {0};
	UB		e[COMM_W_E_SIZE_LIMIT];
	UB		line[96];
	int32_t		rc;
	UW		up, elen;
	ER		er;

	rc = comm_w_unpack_body(body, (UW)size, &b);
	if(rc != COMM_W_OK) {
		/* One visible line, no state change, no sequence number used. */
		malformed++;
		tm_sprintf(line, (UB *)"[comms] intake malformed #%u: kind=%u len=%d (%s)\n",
			   malformed, b.kind, size,
			   (rc == COMM_W_ERR_KIND) ? "reserved kind" : "wrong length");
		tm_printf(line);
		return;
	}

	fold(&b);
	up = now_ms();
	elen = comm_w_copy_body(e, sizeof e, *seq, up, body, (UW)size);
	er = cyw43_utk_mqtt_send(CYW43_MQTT_TOPIC_TELEMETRY, e, elen);
	print_record(kind_name[b.kind], *seq, up, e, elen, er);
	(*seq)++;
}

#if TM_COMMS_E_SELFTEST
LOCAL void selftest_inject(UW n)
{
	static const UB bad[2] = {0x06, 0x00};	/* reserved kind 6 */
	UB	body[COMM_W_INTAKE_SIZE_MAX];
	UW	len;

	switch(n % 7U) {
	case 0: len = comm_w_pack_motion(body, 1, 20, 24, 1500 + 100 * n, -30, 0, 0); break;
	case 1: len = comm_w_pack_line(body, 0, -12); break;
	case 2: len = comm_w_pack_barcode(body, 4, 2); break;
	case 3: len = comm_w_pack_imu(body, 2, 1, 12); break;
	case 4: len = comm_w_pack_obstacle(body, 18, 1); break;
	case 5: len = comm_w_pack_mission(body, 1); break;
	default: comms_tele_send(bad, sizeof bad); return;
	}
	comms_tele_send(body, (INT)len);
}
#endif

LOCAL void comms_task(INT stacd, void *exinf)
{
	UB	rec[COMM_W_H_SIZE];
	UB	body[COMM_W_INTAKE_SIZE_MAX];
	UH	seq = 0;	/* s: one counter for every record this run */
	UW	next, up, len, beats = 0;
	INT	wait, size;
	ER	er;

	(void)stacd; (void)exinf;

	next = now_ms();
	while(1) {
		/* Events until the next heartbeat is due.  An event is published
		   the moment it arrives; the wait is what paces H. */
		wait = (INT)(next - now_ms());
		if(wait > 0) {
			size = tk_rcv_mbf(tele_mbfid, body, (TMO)wait);
			if(size >= 0) {
				handle_intake(body, size, &seq);
				continue;
			}
			if(size != E_TMOUT) {
				tk_dly_tsk((TMO)wait);	/* no queue: just pace H */
			}
			continue;
		}

		s_cache.drp = (UB)((tele_dropped > 15U) ? 15U : tele_dropped);

		up = now_ms();
		len = comm_w_pack_h(rec, seq, up);
		er = cyw43_utk_mqtt_send(CYW43_MQTT_TOPIC_HEARTBEAT, rec, len);
		print_record("H", seq, up, rec, len, er);
		/* s advances even when the record was dropped: a gap in s with
		   the drop counter rising says the car lost it, not the network. */
		seq++;

#if TM_COMMS_E_SELFTEST
		if(beats % 5U == 4U) selftest_inject(beats / 5U);
#endif
		beats++;

		/* Fixed-rate schedule, so the period does not drift.  If we fell
		   behind, restart the schedule. */
		next += H_PERIOD_MS;
		if((INT)(next - now_ms()) <= 0) {
			next = now_ms() + H_PERIOD_MS;
		}
	}
}

EXPORT ER comms_task_start(void)
{
	T_CTSK	ctsk = {0};
	ID	tid;

	ctsk.itskpri = COMMS_PRIORITY;
	ctsk.stksz = COMMS_STACK;
	ctsk.task = (FP)comms_task;
#if TK_SUPPORT_SMP
	ctsk.tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC;
	ctsk.assprc = TP_PRC1;
#else
	ctsk.tskatr = TA_HLNG | TA_RNG3;
#endif
	tid = tk_cre_tsk(&ctsk);
	if(tid < E_OK) return tid;
	return tk_sta_tsk(tid, 0);
}
#else
EXPORT ER comms_task_start(void)
{
	return E_NOSPT;
}
#endif /* TM_WIFI_MQTT */
