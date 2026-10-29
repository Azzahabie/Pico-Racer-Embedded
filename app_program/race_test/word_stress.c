/*
 *	word_stress.c
 *	Stress test for the lock-free shared words (Tribunal 1, Version C row 2).
 *
 *	Several values shared between tasks are published as one aligned 32-bit
 *	word: one writer builds it in a local and stores it once, and readers take
 *	it with one load and no mutex (imu_task's hump_state word and tilt-state
 *	word, nav_ctrl_task's mission_state, Buddy 3's line_state).  Version C
 *	row 2 allows that only with a test showing that a reader never sees a
 *	mixed (torn) value while the writer updates it.  This file is that test.
 *
 *	The writer stores w(n) = n | (~n << 16), n = 1, 2, 3, ...  Every value the
 *	writer stores has its high half equal to the complement of its low half,
 *	so a word made of parts of two different stores fails the check.
 *
 *	Six phases, PHASE_MS each, repeated STRESS_ROUNDS times:
 *
 *	  1 x-core        writer core 1, reader core 0; one store, one load.
 *	                  The tilt-state word and mission_state cross cores.
 *	  2 x-core split  as 1, but the writer stores the two halves separately.
 *	                  CONTROL: must see torn values, or the test is blind.
 *	  3 same w>r      core 1.  Higher-priority writer wakes every 1 ms and
 *	                  stores a burst; lower-priority reader spins.  As
 *	                  line_state: ir_sense_task (10) writes, motion_task (11)
 *	                  reads.
 *	  4 same r>w      core 1.  Lower-priority writer spins; higher-priority
 *	                  reader wakes every 1 ms and reads a burst.  As the
 *	                  hump_state word: imu_task (12) writes, motion_task (11)
 *	                  reads.
 *	  5 same split    as 4, with split stores.  CONTROL.
 *	  6 two loads     as 1, but the reader takes the high half from one load
 *	                  and the low half from a second.  CONTROL: why a reader
 *	                  must load the word once and take every field from that
 *	                  copy.
 *
 *	Every test task is below every car task in priority, so the car's tasks,
 *	their interrupts and the radio keep running on both cores and add their
 *	load.  Built only when TM_WORD_STRESS is 1 (word_stress.h).
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

/* app_program/ is not on INCPATH; see race_test.c. */
#include "../usb_console_compat.h"
#include "word_stress.h"

#if TM_WORD_STRESS

#if !TK_SUPPORT_SMP
#error "word_stress needs a two-core build (SMP=1)"
#endif

/* ------------------------------------------------------------------ *
 *  Tunables
 * ------------------------------------------------------------------ */

#define STRESS_STACK_SZ	4096
#define PHASE_MS	10000	/* each phase                                */
#define STRESS_ROUNDS	3	/* all six phases, this many times           */
#define BURST		200	/* stores or reads per 1 ms wake             */
#define DONE_TMO_MS	2000	/* wait for both workers after stop          */

/* Below every car task (motion_report is 24, the lowest).  The controller
   sleeps almost all the time; the periodic worker sits above the spinning
   one so that it preempts it. */
#define PRI_CTRL	25
#define PRI_HIGH	26	/* the 1 ms burst worker                     */
#define PRI_LOW		28	/* the spinning worker                       */

#define HALF_MASK	0x0000FFFFU

/* ------------------------------------------------------------------ *
 *  The shared word
 *
 *  A union so the split-store controls can write one half at a time.
 *  Little-endian: h[0] is the low half, h[1] the high half.
 * ------------------------------------------------------------------ */

LOCAL volatile union {
	UW	w;
	UH	h[2];
} g_shared;

#define W_INITIAL	0xFFFF0000U	/* w(0): valid                       */

/* A whole value from one store: high half is the complement of the low. */
LOCAL BOOL word_ok(UW w)
{
	return (((w >> 16) ^ (w & HALF_MASK)) == HALF_MASK) ? TRUE : FALSE;
}

LOCAL UW word_of(UW n)
{
	return (n & HALF_MASK) | ((~n & HALF_MASK) << 16);
}

/* ------------------------------------------------------------------ *
 *  Control between the controller and the two workers
 * ------------------------------------------------------------------ */

LOCAL ID		g_flgid;
#define DONE_W		(1U << 0)
#define DONE_R		(1U << 1)

LOCAL volatile BOOL	g_stop;

/* Reader results, written by the reader before it sets DONE_R. */
LOCAL volatile UW	g_reads;
LOCAL volatile UW	g_changes;
LOCAL volatile UW	g_torn;
LOCAL volatile UW	g_example;	/* first torn value seen             */

/* Writer result: stores made. */
LOCAL volatile UW	g_writes;

LOCAL void worker_done(UINT bit)
{
	tk_set_flg(g_flgid, bit);
	tk_exd_tsk();
}

/* ------------------------------------------------------------------ *
 *  Writers
 * ------------------------------------------------------------------ */

/* One store per value, as fast as the core allows. */
LOCAL void writer_spin(INT stacd, void *exinf)
{
	UW	n = 0;
	UW	v;

	(void)stacd; (void)exinf;

	while(!g_stop) {
		n++;
		v = word_of(n);			/* build in a local ...  */
		g_shared.w = v;			/* ... one 32-bit store  */
	}
	g_writes = n;
	worker_done(DONE_W);
}

/* The two halves as two 16-bit stores (the controls). */
LOCAL void writer_spin_split(INT stacd, void *exinf)
{
	UW	n = 0;

	(void)stacd; (void)exinf;

	while(!g_stop) {
		n++;
		g_shared.h[0] = (UH)n;
		g_shared.h[1] = (UH)~n;
	}
	g_writes = n;
	worker_done(DONE_W);
}

/* A burst of single stores every 1 ms, at the higher priority. */
LOCAL void writer_burst(INT stacd, void *exinf)
{
	UW	n = 0;
	UW	v;
	INT	k;

	(void)stacd; (void)exinf;

	while(!g_stop) {
		for(k = 0; k < BURST; k++) {
			n++;
			v = word_of(n);
			g_shared.w = v;
		}
		tk_dly_tsk(1);
	}
	g_writes = n;
	worker_done(DONE_W);
}

/* ------------------------------------------------------------------ *
 *  Readers
 * ------------------------------------------------------------------ */

LOCAL void reader_publish(UW reads, UW changes, UW torn, UW example)
{
	g_reads = reads;
	g_changes = changes;
	g_torn = torn;
	g_example = example;
	worker_done(DONE_R);
}

/* One load per read, as fast as the core allows. */
LOCAL void reader_spin(INT stacd, void *exinf)
{
	UW	prev = g_shared.w;
	UW	w;
	UW	reads = 0, changes = 0, torn = 0, example = 0;

	(void)stacd; (void)exinf;

	while(!g_stop) {
		w = g_shared.w;			/* one 32-bit load */
		reads++;
		if(!word_ok(w)) {
			if(torn == 0) example = w;
			torn++;
		}
		if(w != prev) {
			changes++;
			prev = w;
		}
	}
	reader_publish(reads, changes, torn, example);
}

/* A burst of single loads every 1 ms, at the higher priority. */
LOCAL void reader_burst(INT stacd, void *exinf)
{
	UW	prev = g_shared.w;
	UW	w;
	UW	reads = 0, changes = 0, torn = 0, example = 0;
	INT	k;

	(void)stacd; (void)exinf;

	while(!g_stop) {
		for(k = 0; k < BURST; k++) {
			w = g_shared.w;
			reads++;
			if(!word_ok(w)) {
				if(torn == 0) example = w;
				torn++;
			}
			if(w != prev) {
				changes++;
				prev = w;
			}
		}
		tk_dly_tsk(1);
	}
	reader_publish(reads, changes, torn, example);
}

/* Two loads, one field from each: what two IMU_W_GET(imu_word_read(), ...)
   calls in a row would do. */
LOCAL void reader_twice(INT stacd, void *exinf)
{
	UW	prev = g_shared.w;
	UW	a, b, w;
	UW	reads = 0, changes = 0, torn = 0, example = 0;

	(void)stacd; (void)exinf;

	while(!g_stop) {
		a = g_shared.w;
		b = g_shared.w;
		w = (a & ~HALF_MASK) | (b & HALF_MASK);
		reads++;
		if(!word_ok(w)) {
			if(torn == 0) example = w;
			torn++;
		}
		if(w != prev) {
			changes++;
			prev = w;
		}
	}
	reader_publish(reads, changes, torn, example);
}

/* ------------------------------------------------------------------ *
 *  Phases
 * ------------------------------------------------------------------ */

typedef struct {
	const char	*name;
	FP		writer;
	PRI		wpri;
	UINT		wprc;
	FP		reader;
	PRI		rpri;
	UINT		rprc;
	BOOL		control;	/* expected to see torn values */
} phase_t;

LOCAL const phase_t g_phases[] = {
	{ "1 x-core      ", (FP)writer_spin,       PRI_LOW,  TP_PRC2,
			    (FP)reader_spin,       PRI_LOW,  TP_PRC1, FALSE },
	{ "2 x-core split", (FP)writer_spin_split, PRI_LOW,  TP_PRC2,
			    (FP)reader_spin,       PRI_LOW,  TP_PRC1, TRUE  },
	{ "3 same w>r    ", (FP)writer_burst,      PRI_HIGH, TP_PRC2,
			    (FP)reader_spin,       PRI_LOW,  TP_PRC2, FALSE },
	{ "4 same r>w    ", (FP)writer_spin,       PRI_LOW,  TP_PRC2,
			    (FP)reader_burst,      PRI_HIGH, TP_PRC2, FALSE },
	{ "5 same split  ", (FP)writer_spin_split, PRI_LOW,  TP_PRC2,
			    (FP)reader_burst,      PRI_HIGH, TP_PRC2, TRUE  },
	{ "6 two loads   ", (FP)writer_spin,       PRI_LOW,  TP_PRC2,
			    (FP)reader_twice,      PRI_LOW,  TP_PRC1, TRUE  },
};

#define PHASE_COUNT	(sizeof(g_phases) / sizeof(g_phases[0]))

LOCAL ER start_worker(FP task, PRI pri, UINT prc)
{
	T_CTSK	ctsk;
	ID	tid;

	ctsk.exinf = NULL;
	ctsk.tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC;
	ctsk.task = task;
	ctsk.itskpri = pri;
	ctsk.stksz = STRESS_STACK_SZ;
	ctsk.assprc = prc;

	tid = tk_cre_tsk(&ctsk);
	if(tid <= E_OK) return (ER)tid;
	return tk_sta_tsk(tid, 0);
}

/* Run one phase and print one line.  Returns TRUE if the phase gave the
   expected result (no torn value with coverage, or a control that saw
   tearing). */
LOCAL BOOL run_phase(const phase_t *p)
{
	UINT	ptn;
	ER	er;
	BOOL	good;
	UB	line[160];
	const char *verdict;

	g_shared.w = W_INITIAL;
	g_stop = FALSE;
	g_reads = g_changes = g_torn = g_example = g_writes = 0;
	tk_clr_flg(g_flgid, ~(DONE_W | DONE_R));

	er = start_worker(p->writer, p->wpri, p->wprc);
	if(er >= E_OK) er = start_worker(p->reader, p->rpri, p->rprc);
	if(er < E_OK) {
		tm_printf((UB *)"[stress] phase %s: task start failed, er=%d\n",
			  p->name, er);
		g_stop = TRUE;
		return FALSE;
	}

	tk_dly_tsk(PHASE_MS);
	g_stop = TRUE;

	er = tk_wai_flg(g_flgid, DONE_W | DONE_R, TWF_ANDW, &ptn, DONE_TMO_MS);
	if(er != E_OK) {
		tm_printf((UB *)"[stress] phase %s: workers did not stop, er=%d\n",
			  p->name, er);
		return FALSE;
	}

	if(p->control) {
		good = (g_torn > 0) ? TRUE : FALSE;
		verdict = good ? "control OK (tearing seen)"
			       : "CONTROL SAW NOTHING: test is blind";
	} else if(g_torn > 0) {
		good = FALSE;
		verdict = "FAIL: torn value";
	} else if(g_changes == 0) {
		good = FALSE;
		verdict = "NO COVERAGE: reader never saw a change";
	} else {
		good = TRUE;
		verdict = "PASS";
	}

	tm_sprintf(line, (UB *)"[stress] %s  writes=%u reads=%u changes=%u"
			       " torn=%u first=0x%08x  %s\n",
		   p->name, g_writes, g_reads, g_changes, g_torn, g_example,
		   verdict);
	tm_printf((UB *)"%s", line);
	return good;
}

/* ------------------------------------------------------------------ *
 *  Controller
 * ------------------------------------------------------------------ */

LOCAL void stress_ctrl_task(INT stacd, void *exinf)
{
	INT	round;
	UW	i;
	UW	good, all_good = 0;
	INT	k;

	(void)stacd; (void)exinf;

	for(k = 0; k < 200 && tm_usb_state() < 2; k++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(5000);	/* let boot, calibration and Wi-Fi settle */

	tm_printf((UB *)"\n=== word stress test: lock-free shared words"
			" (Tribunal 1 C row 2) ===\n");
	tm_printf((UB *)"[stress] %u phases x %u ms, %u rounds; phases 2, 5, 6"
			" are controls that must see tearing\n",
		  (UINT)PHASE_COUNT, PHASE_MS, STRESS_ROUNDS);

	for(round = 1; round <= STRESS_ROUNDS; round++) {
		good = 0;
		tm_printf((UB *)"[stress] round %d\n", round);
		for(i = 0; i < PHASE_COUNT; i++) {
			if(run_phase(&g_phases[i])) good++;
		}
		tm_printf((UB *)"[stress] round %d: %u of %u phases as expected\n",
			  round, good, (UINT)PHASE_COUNT);
		all_good += good;
	}

	tm_printf((UB *)"[stress] done: %u of %u phase runs as expected -> %s\n",
		  all_good, (UINT)(PHASE_COUNT * STRESS_ROUNDS),
		  (all_good == PHASE_COUNT * STRESS_ROUNDS)
		  ? "RESULT PASS" : "RESULT FAIL (see the lines above)");

	tk_slp_tsk(TMO_FEVR);
}

LOCAL T_CTSK ctsk_stress_ctrl = {
	.itskpri	= PRI_CTRL,
	.stksz		= STRESS_STACK_SZ,
	.task		= stress_ctrl_task,
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC1,	/* core 0: it prints */
};

LOCAL T_CFLG cflg_stress = {
	.flgatr		= TA_TFIFO | TA_WMUL,
	.iflgptn	= 0,
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

EXPORT ER word_stress_start(void)
{
	ID	tid;

	g_flgid = tk_cre_flg(&cflg_stress);
	if(g_flgid <= E_OK) return g_flgid;

	tid = tk_cre_tsk(&ctsk_stress_ctrl);
	if(tid <= E_OK) return tid;
	return tk_sta_tsk(tid, 0);
}

#endif /* TM_WORD_STRESS */
