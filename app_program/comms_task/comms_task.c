/*
 *	comms_task.c
 *	Buddy 1: car state -> MQTT records (docs/comms/README.md).
 *
 *	First slice: the H heartbeat.  Once a second it packs a 7-byte H
 *	record (MQTT_SCHEMA.md section 6) and queues it for the radio task with
 *	cyw43_utk_mqtt_send().  It never touches lwIP, and never waits on the
 *	network: a full queue drops the record and the console says so.
 *
 *	Priority 16, one below nav_ctrl_task (Week 6 report), pinned to
 *	processor 1 (RP2040 core 0) with the radio, away from the control tasks
 *	on core 1.  The core is proposed, still being agreed with Buddy 2.
 *
 *	Every .c under app_program/ is compiled in every build, so the body is
 *	gated on the MQTT build; nothing calls comms_task_start() otherwise.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "comms_task.h"
#include "../common/messages.h"

#if TM_WIFI_MQTT
#include "cyw43_utk.h"

#define COMMS_PRIORITY	16
#define COMMS_STACK	4096
#define H_PERIOD_MS	1000	/* the S cadence (NFR-10); H rides on it */

/* ms since boot.  The low word wraps after ~49.7 days (design section 4.1). */
LOCAL UW now_ms(void)
{
	SYSTIM	t;

	tk_get_otm(&t);
	return t.lo;
}

LOCAL void comms_task(INT stacd, void *exinf)
{
	UB	rec[MSG_H_LEN];
	UB	line[96];
	UB	*p;
	UH	seq = 0;	/* s: one counter for every record this run */
	UW	next, up, len, i;
	INT	wait;
	ER	er;

	(void)stacd; (void)exinf;

	next = now_ms();
	while(1) {
		up = now_ms();
		len = msg_pack_h(rec, seq, up);
		er = cyw43_utk_mqtt_send(CYW43_MQTT_TOPIC_HEARTBEAT, rec, len);

		/* The bytes are printed so the console can be checked against
		   the laptop decoder.  Compose, then one tm_printf. */
		p = line;
		p += tm_sprintf(p, (UB *)"[comms] H s=%u up=%u bytes=", seq, up);
		for(i = 0; i < len; i++) {
			p += tm_sprintf(p, (UB *)"%02x", rec[i]);
		}
		if(er == E_OK) {
			tm_sprintf(p, (UB *)" queued\n");
		} else {
			tm_sprintf(p, (UB *)" dropped(%d)\n", er);
		}
		tm_printf(line);

		/* s advances even when the record was dropped: a gap in s with
		   the drop counter rising says the car lost it, not the network. */
		seq++;

		/* Fixed-rate schedule, so the period does not drift by the time
		   spent above.  If we fell behind, restart the schedule. */
		next += H_PERIOD_MS;
		wait = (INT)(next - now_ms());
		if(wait > 0) {
			tk_dly_tsk((TMO)wait);
		} else {
			next = now_ms();
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
