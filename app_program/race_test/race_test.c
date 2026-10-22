/*
 *	race_test.c
 *	A two-core demonstration of the shared packed-word hazard.
 *
 *	Change Requirement 1 says that when packed data is shared between tasks,
 *	or between an ISR and a task, access must be safe and a read-modify-write
 *	must not race.  This file shows both halves of that on the board.
 *
 *	Two tasks, one pinned to each processor, each bump their own 16-bit half
 *	of ONE 32-bit word.  Writing one half rewrites the whole word, so without
 *	synchronisation the two cores overwrite each other's increments.  Phase 1
 *	runs the loop with no lock and the two halves disagree.  Phase 2 runs the
 *	same loop under the kernel mutex and the halves are exact.
 *
 *	Built only when TM_RACE_TEST is 1 (race_test.h).  While it is 0 this file
 *	compiles to nothing, so a normal image is unchanged.
 *
 *	Build with two cores, for example:
 *	    make SMP=1 CONSOLE=usb_cdc -j8
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

/* The console-state accessors live in the parent folder.  app_program/ is not
   on INCPATH (build_make/makefile only adds include, config and knlinc), so
   this one is reached by a relative path. */
#include "../usb_console_compat.h"
#include "race_test.h"

#if TM_RACE_TEST

/* ------------------------------------------------------------------ *
 *  Tunables
 * ------------------------------------------------------------------ */

#define RACE_STACK_SZ	4096
/* Each half is 16 bits, so it caps at 65535.  Keep the count under that or a
   perfect run wraps and the arithmetic below lies. */
#define RACE_ITER	60000U

#define RACE_PRIO_A	11
#define RACE_PRIO_B	12
#define RACE_PRIO_REP	13

/* ------------------------------------------------------------------ *
 *  Shared state
 *
 *  volatile keeps the loop honest: without it the compiler may cache the word
 *  in a register and collapse the whole loop into one increment.  It does NOT
 *  make the access safe.  PORT_RP2040.md safety rule 3 says volatile alone is
 *  not inter-core synchronisation, which is exactly what phase 1 shows.
 * ------------------------------------------------------------------ */

LOCAL volatile UW	unsafe_word;	/* phase 1: no lock            */
LOCAL volatile UW	safe_word;	/* phase 2: under the mutex     */

#define HALF_MASK	0xFFFFU
#define LOW_MASK	0x0000FFFFU
#define HIGH_MASK	0xFFFF0000U

LOCAL ID		race_mtxid;
LOCAL ID		race_flgid;

#define RACE_START	(1U << 0)	/* reporter -> both workers      */
#define RACE_DONE_A	(1U << 1)
#define RACE_DONE_B	(1U << 2)

/* ------------------------------------------------------------------ *
 *  The two halves
 * ------------------------------------------------------------------ */

/* Bump the high half of a word and leave the low half alone.  A plain read,
   modify, write, which is the pattern the requirement is about. */
LOCAL UW bump_high(UW word)
{
	return (word & LOW_MASK) | ((((word >> 16) + 1U) & HALF_MASK) << 16);
}

/* Bump the low half of a word and leave the high half alone. */
LOCAL UW bump_low(UW word)
{
	return (word & HIGH_MASK) | (((word & LOW_MASK) + 1U) & HALF_MASK);
}

/* ------------------------------------------------------------------ *
 *  Worker A -- the high half, pinned to processor 1
 * ------------------------------------------------------------------ */

LOCAL void race_task_a(INT stacd, void *exinf)
{
	UINT	ptn;
	UW	i;

	(void)stacd; (void)exinf;

	/* Line up with worker B so the two loops really do overlap.  The bit
	   stays set, so arriving late still gets straight through. */
	tk_wai_flg(race_flgid, RACE_START, TWF_ORW, &ptn, TMO_FEVR);

	/* Phase 1: no lock. */
	for(i = 0; i < RACE_ITER; i++) {
		unsafe_word = bump_high(unsafe_word);
	}

	/* Phase 2: the same loop, serialised by the kernel mutex. */
	for(i = 0; i < RACE_ITER; i++) {
		tk_loc_mtx(race_mtxid, TMO_FEVR);
		safe_word = bump_high(safe_word);
		tk_unl_mtx(race_mtxid);
	}

	tk_set_flg(race_flgid, RACE_DONE_A);
	tk_slp_tsk(TMO_FEVR);		/* done: stop using the core */
}

/* ------------------------------------------------------------------ *
 *  Worker B -- the low half, pinned to processor 2
 * ------------------------------------------------------------------ */

LOCAL void race_task_b(INT stacd, void *exinf)
{
	UINT	ptn;
	UW	i;

	(void)stacd; (void)exinf;

	tk_wai_flg(race_flgid, RACE_START, TWF_ORW, &ptn, TMO_FEVR);

	/* Phase 1: no lock. */
	for(i = 0; i < RACE_ITER; i++) {
		unsafe_word = bump_low(unsafe_word);
	}

	/* Phase 2: the same loop, serialised by the kernel mutex. */
	for(i = 0; i < RACE_ITER; i++) {
		tk_loc_mtx(race_mtxid, TMO_FEVR);
		safe_word = bump_low(safe_word);
		tk_unl_mtx(race_mtxid);
	}

	tk_set_flg(race_flgid, RACE_DONE_B);
	tk_slp_tsk(TMO_FEVR);
}

/* ------------------------------------------------------------------ *
 *  Reporter -- starts the race, then prints the four counts
 * ------------------------------------------------------------------ */

LOCAL void race_report_task(INT stacd, void *exinf)
{
	UINT	ptn;
	UW	ua, ub, sa, sb;
	ER	er;
	INT	i;

	(void)stacd; (void)exinf;

	/* Wait for the console the same way the other tasks do, so the result
	   is not printed before the host is reading. */
	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(1000);

	tm_printf((UB *)"\n=== race test: one packed word, two cores ===\n");
	tm_printf((UB *)"each core bumps one half %u times, so each half should reach %u\n",
		  RACE_ITER, RACE_ITER);

	tk_set_flg(race_flgid, RACE_START);

	er = tk_wai_flg(race_flgid, RACE_DONE_A | RACE_DONE_B,
			TWF_ANDW | TWF_CLR, &ptn, 60000);
	if(er != E_OK) {
		tm_printf((UB *)"race test: workers did not finish, er=%d\n", er);
		tk_slp_tsk(TMO_FEVR);
	}

	ua = (unsafe_word & HIGH_MASK) >> 16;
	ub = unsafe_word & LOW_MASK;
	sa = (safe_word & HIGH_MASK) >> 16;
	sb = safe_word & LOW_MASK;

	tm_printf((UB *)"phase 1 no lock : high=%u low=%u  loss=%u\n",
		  ua, ub, (RACE_ITER - ua) + (RACE_ITER - ub));
	tm_printf((UB *)"phase 2 mutex   : high=%u low=%u  loss=%u\n",
		  sa, sb, (RACE_ITER - sa) + (RACE_ITER - sb));

	if(ua < RACE_ITER || ub < RACE_ITER) {
		tm_printf((UB *)"result: the unsynchronised read-modify-write lost updates\n");
	} else {
		tm_printf((UB *)"result: no loss in phase 1, run again or raise RACE_ITER\n");
	}

	tk_slp_tsk(TMO_FEVR);
}

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 * ------------------------------------------------------------------ */

LOCAL T_CTSK ctsk_race_a = {
	.itskpri	= RACE_PRIO_A,
	.stksz		= RACE_STACK_SZ,
	.task		= race_task_a,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC1,
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_race_b = {
	.itskpri	= RACE_PRIO_B,
	.stksz		= RACE_STACK_SZ,
	.task		= race_task_b,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC2,
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_race_rep = {
	.itskpri	= RACE_PRIO_REP,
	.stksz		= RACE_STACK_SZ,
	.task		= race_report_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL T_CMTX cmtx_race = {
	.mtxatr		= TA_TFIFO | TA_INHERIT,
};

LOCAL T_CFLG cflg_race = {
	.flgatr		= TA_TFIFO | TA_WMUL,
	.iflgptn	= 0,
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

EXPORT ER race_test_start(void)
{
	ID	tid;

	race_mtxid = tk_cre_mtx(&cmtx_race);
	if(race_mtxid <= E_OK) return race_mtxid;

	race_flgid = tk_cre_flg(&cflg_race);
	if(race_flgid <= E_OK) return race_flgid;

	tid = tk_cre_tsk(&ctsk_race_rep);
	if(tid <= E_OK) return tid;
	tk_sta_tsk(tid, 0);

	tid = tk_cre_tsk(&ctsk_race_a);
	if(tid <= E_OK) return tid;
	tk_sta_tsk(tid, 0);

	tid = tk_cre_tsk(&ctsk_race_b);
	if(tid <= E_OK) return tid;
	tk_sta_tsk(tid, 0);

	return E_OK;
}

#endif	/* TM_RACE_TEST */