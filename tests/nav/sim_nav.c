/* Host test for nav_ctrl_task's junction handling (app_program/nav_task/
 * nav_ctrl.c, FR-08): the real nav_ctrl.c is compiled in with the motion
 * sim's kernel stubs and small fakes for the board, barcode_task and
 * comms_task.  make -C tests/nav
 *
 * What it checks: the barcode command goes to motion_task once per
 * junction, and is used up only when motion turns (MOVE_DONE); a refusal
 * (CMD_REJECTED, motion not in Line Following, e.g. during a bypass) keeps
 * it for the next junction.  Calibration: a GP20 fault reset waits
 * NAV_CAL_SETTLE_MS before CAL_REQ; boot and an MQTT reset do not; an
 * MQTT command sent while calibrating is dropped, not applied at Ready. */

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../../app_program/nav_task/nav_ctrl.c"

/* ---- checks ---------------------------------------------------------- */
static int g_checks;
static int g_failures;
#define CHECK(c)                                                              \
	do {                                                                  \
		g_checks++;                                                   \
		if (!(c)) {                                                   \
			g_failures++;                                         \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);   \
		}                                                             \
	} while (0)

/* ---- fakes: kernel --------------------------------------------------- */
static UINT g_flg[8];			/* event flags by ID 1..7 */
static ID   g_next_flg = 1;
static UB   g_sent[16][2];		/* messages on nav_cmd_mbf */
static int  g_sent_n;
static int  g_mbf_full;			/* tk_snd_mbf returns E_TMOUT */
#define NAV_CMD_MBF_ID	50
#define IR_FLG_ID	60		/* ir_sense_task's flag (CAL_REQ) */
#define IMU_FLG_ID	61		/* imu_task's flag (CAL_REQ) */
#define CAL_REQ_BIT	0x0001U
static long g_slept_ms;			/* every tk_dly_tsk so far */
static long g_cal_req_ms;		/* g_slept_ms at the first CAL_REQ, -1 = none */
static int  g_cal_fail;			/* the tasks answer CAL_FAIL */
static jmp_buf g_main_loop;		/* nav_ctrl_task reached its main wait */
static int  g_in_task;
static UB   g_mq_during_cal;		/* MQTT command sent mid-calibration, 0 = none */
static void mq_put(UB cmd);

/* ir_sense_task and imu_task answer a CAL_REQ at once. */
static void cal_answer(ID id)
{
	if (g_cal_req_ms < 0) {
		g_cal_req_ms = g_slept_ms;
	}
	if (0U != g_mq_during_cal) {
		mq_put(g_mq_during_cal);	/* comms_task, while nav calibrates */
		g_mq_during_cal = 0U;
	}
	g_flg[g_nav_flgid] |= g_cal_fail ? NAV_FLG_CAL_FAIL
					 : ((IR_FLG_ID == id) ? NAV_FLG_CAL_OK_IR
							      : NAV_FLG_CAL_OK_IMU);
}

ID tk_cre_flg(CONST T_CFLG *p) { (void)p; return g_next_flg++; }
ER tk_set_flg(ID id, UINT ptn)
{
	if ((IR_FLG_ID == id) || (IMU_FLG_ID == id)) {
		if (0U != (ptn & CAL_REQ_BIT)) {
			cal_answer(id);
		}
		return E_OK;
	}
	g_flg[id] |= ptn;
	return E_OK;
}
ER tk_clr_flg(ID id, UINT ptn) { g_flg[id] &= ptn; return E_OK; }
ER tk_ref_flg(ID id, T_RFLG *p) { p->flgptn = g_flg[id]; return E_OK; }
ER tk_wai_flg(ID id, UINT ptn, UINT mode, UINT *p_ptn, TMO tmo)
{
	(void)tmo;
	if (g_in_task && (NAV_WAIT_BITS == ptn)) {
		longjmp(g_main_loop, 1);	/* boot is over: leave the task */
	}
	*p_ptn = g_flg[id] & ptn;
	if (0U != (mode & TWF_BITCLR)) {
		g_flg[id] &= ~*p_ptn;
	}
	return (0U != *p_ptn) ? E_OK : E_TMOUT;
}
ER tk_snd_mbf(ID id, void *msg, INT size, TMO tmo)
{
	(void)tmo;
	if ((NAV_CMD_MBF_ID != id) || (2 != size) || g_mbf_full) {
		return E_TMOUT;
	}
	memcpy(g_sent[g_sent_n % 16], msg, 2);
	g_sent_n++;
	return E_OK;
}

/* mission_cmd_mbf: comms_task's MQTT commands, one byte each */
#define MISSION_MBF_ID	51
static UB  g_mq[8];
static int g_mq_head;
static int g_mq_n;
static void mq_put(UB cmd)
{
	g_mq[(g_mq_head + g_mq_n) % 8] = cmd;
	g_mq_n++;
	g_flg[g_nav_flgid] |= NAV_FLG_MISSION_CMD;
}
INT tk_rcv_mbf(ID id, void *msg, TMO tmo)
{
	(void)tmo;
	if ((MISSION_MBF_ID != id) || (0 == g_mq_n)) {
		return E_TMOUT;
	}
	*(UB *)msg = g_mq[g_mq_head];
	g_mq_head = (g_mq_head + 1) % 8;
	g_mq_n--;
	return 1;
}
ER tk_dly_tsk(TMO tmo) { g_slept_ms += tmo; return E_OK; }
ER tk_slp_tsk(TMO tmo) { (void)tmo; return E_OK; }

static char g_log[8192];
INT tm_printf(const UB *format, ...)
{
	va_list ap;
	size_t  len = strlen(g_log);

	va_start(ap, format);
	(void)vsnprintf(g_log + len, sizeof(g_log) - len, (const char *)format, ap);
	va_end(ap);
	return 0;
}

/* ---- fakes: board ---------------------------------------------------- */
UW   in_w(UW addr) { (void)addr; return 0xFFFFFFFFu; }
void out_w(UW addr, UW data) { (void)addr; (void)data; }
void clr_w(UW addr, UW data) { (void)addr; (void)data; }
ER   gpio_set_pin(UINT no, UINT mode) { (void)no; (void)mode; return E_OK; }
ER   gpio_set_val(UINT no, UINT val) { (void)no; (void)val; return E_OK; }
static UINT g_pins_low;			/* buttons held down, bit = GP */
UINT gpio_get_val(UINT no) { return (0U != (g_pins_low & (1U << no))) ? 0U : 1U; }
ER   pwm_set_pin(UINT no) { (void)no; return E_OK; }
ER   pwm_set_enabled(UINT no, BOOL en) { (void)no; (void)en; return E_OK; }
ER   pwm_set_wrap(UINT no, UW wrap) { (void)no; (void)wrap; return E_OK; }
ER   pwm_set_cc(UINT no, UW cc) { (void)no; (void)cc; return E_OK; }
ER   gpio_irq_init(void) { return E_OK; }
ER   gpio_irq_attach(UINT pin, UW edges, gpio_isr_t p_isr) { (void)pin; (void)edges; (void)p_isr; return E_OK; }

/* ---- fakes: barcode_task and comms_task ------------------------------ */
static uint32_t g_bar_word = BARCODE_CMD_NONE;
uint32_t barcode_cmd_word(void) { return g_bar_word; }
ER comms_tele_send(const UB *body, INT length) { (void)body; (void)length; return E_OK; }

/* barcode_task's store: letter, action, sequence */
static void bar_store(uint32_t letter, uint32_t action, uint32_t seq)
{
	g_bar_word = barcode_cmd_pack(letter, action, seq);
}

/* ---- helpers --------------------------------------------------------- */
static void reset(void)
{
	nav_bind_t bind;

	memset(g_flg, 0, sizeof(g_flg));
	g_next_flg  = 1;
	g_nav_flgid = 0;
	g_fault_flgid = 0;
	(void)nav_ctrl_init();
	memset(&bind, 0, sizeof(bind));
	bind.nav_cmd_mbfid = NAV_CMD_MBF_ID;
	bind.ir_flgid = IR_FLG_ID;
	bind.imu_flgid = IMU_FLG_ID;
	bind.cal_req_bit = CAL_REQ_BIT;
	bind.mission_cmd_mbfid = MISSION_MBF_ID;
	nav_bind(&bind);
	g_sent_n = 0;
	g_mbf_full = 0;
	g_slept_ms = 0;
	g_cal_req_ms = -1;
	g_cal_fail = 0;
	g_in_task = 0;
	g_pins_low = 0U;
	g_mq_head = 0;
	g_mq_n = 0;
	g_mq_during_cal = 0U;
	g_bar_word = BARCODE_CMD_NONE;
	g_cmd_taken = 0U;
	g_move_open = false;
	g_cmd_sent = false;
	g_log[0] = '\0';
	g_mission = (uint32_t)MS_RUNNING;
}

/* the last JUNCTION nav sent is this action */
static bool sent_junction(uint32_t action)
{
	return (0 < g_sent_n) && (NAV_CMD_JUNCTION == g_sent[(g_sent_n - 1) % 16][0])
	       && (action == g_sent[(g_sent_n - 1) % 16][1]);
}

/* ---- scenarios ------------------------------------------------------- */

/* The bug: a junction motion refuses (here during a bypass) used to use up
   the barcode command, so the real junction after it went straight on. */
static void refused_keeps_command(void)
{
	reset();
	bar_store(11u, 1u, 1u);			/* 'L', LEFT, seq 1 */
	on_junction();				/* GP7 crosses a line mid-bypass */
	CHECK(1 == g_sent_n);
	CHECK(sent_junction(1u));
	on_motion_reply(NAV_FLG_CMD_REJECTED);	/* motion was in BYPASS */
	CHECK(NULL != strstr(g_log, "CMD_REJECTED, barcode command kept for the next junction"));
	CHECK(barcode_cmd_pending(g_bar_word, g_cmd_taken));
	on_junction();				/* the real junction */
	CHECK(2 == g_sent_n);
	CHECK(sent_junction(1u));		/* still LEFT */
	on_motion_reply(NAV_FLG_MOVE_DONE);	/* turned */
	CHECK(!barcode_cmd_pending(g_bar_word, g_cmd_taken));
	on_junction();				/* the next junction: no command */
	CHECK(2 == g_sent_n);
	CHECK(NULL != strstr(g_log, "no barcode command: straight on"));
}

/* MOVE_DONE uses it up at once, as before. */
static void done_uses_command(void)
{
	reset();
	bar_store(17u, 2u, 5u);			/* 'R', RIGHT, seq 5 */
	on_junction();
	CHECK(sent_junction(2u));
	on_motion_reply(NAV_FLG_MOVE_DONE);
	CHECK(!barcode_cmd_pending(g_bar_word, g_cmd_taken));
	CHECK(NULL == strstr(g_log, "kept"));
}

/* A new barcode read while the turn runs is for the next junction. */
static void newer_command_waits(void)
{
	reset();
	bar_store(11u, 1u, 1u);			/* LEFT, seq 1 */
	on_junction();
	bar_store(20u, 3u, 2u);			/* 'U', U_TURN, seq 2, read mid-turn */
	on_motion_reply(NAV_FLG_MOVE_DONE);	/* the LEFT is done */
	CHECK(barcode_cmd_pending(g_bar_word, g_cmd_taken));
	on_junction();
	CHECK(sent_junction(3u));		/* the U-turn, at the next junction */
}

/* A junction while a move is open is ignored; a full buffer sends nothing
   and keeps the command; a mission change drops the open move. */
static void edges(void)
{
	reset();
	bar_store(11u, 1u, 1u);
	on_junction();
	on_junction();				/* during the turn */
	CHECK(1 == g_sent_n);
	CHECK(NULL != strstr(g_log, "junction during a move: ignored"));

	reset();
	bar_store(11u, 1u, 1u);
	g_mbf_full = 1;
	on_junction();
	CHECK(0 == g_sent_n);
	CHECK(!g_move_open);
	CHECK(barcode_cmd_pending(g_bar_word, g_cmd_taken));
	g_mbf_full = 0;
	on_junction();
	CHECK(sent_junction(1u));

	reset();
	bar_store(11u, 1u, 1u);
	on_junction();
	set_state(MS_FINISHED);			/* the open move ends with it */
	CHECK(!g_move_open && !g_cmd_sent);
	on_motion_reply(NAV_FLG_MOVE_DONE);	/* a late reply uses nothing */
	CHECK(barcode_cmd_pending(g_bar_word, g_cmd_taken));

	reset();
	g_mission = (uint32_t)MS_PAUSED;
	bar_store(11u, 1u, 1u);
	on_junction();
	CHECK(0 == g_sent_n);			/* not Running: ignored */
}

/* text a appears in the log, and before text b */
static bool log_before(const char *a, const char *b)
{
	const char *p_a = strstr(g_log, a);
	const char *p_b = strstr(g_log, b);

	return (NULL != p_a) && (NULL != p_b) && (p_a < p_b);
}

/* Boot: run nav_ctrl_task up to its main wait. */
static void run_boot(void)
{
	g_mission = (uint32_t)MS_INITIALISING;
	g_in_task = 1;
	if (0 == setjmp(g_main_loop)) {
		nav_ctrl_task(0, NULL);
	}
	g_in_task = 0;
}

/* N1: a GP20 fault reset waits, hands off, before CAL_REQ (the press
   rocks the car); FAULT -> CALIBRATING is shown first. */
static void fault_reset_waits(void)
{
	reset();
	g_mission = (uint32_t)MS_FAULT;
	g_pins_low = 1U << BTN_START_PIN;
	on_button();
	CHECK(MS_READY == nav_mission_state());
	CHECK(g_cal_req_ms >= (long)(BTN_DEBOUNCE_MS + NAV_CAL_SETTLE_MS));
	CHECK(log_before("mission FAULT -> CALIBRATING", "hands off the car"));
	CHECK(log_before("hands off the car", "mission CALIBRATING -> READY"));
	CHECK((2 == g_sent_n) && (NAV_CMD_FAULT_CLEAR == g_sent[0][0]) &&
	      (FAULT_CLEAR_IMU == g_sent[0][1]) && (FAULT_CLEAR_MOTION == g_sent[1][1]));
}

/* N1: an MQTT reset and the boot calibration do not wait. */
static void reset_and_boot_no_wait(void)
{
	reset();
	g_mission = (uint32_t)MS_FAULT;
	mq_put((UB)MCMD_RESET);
	on_mission_cmd();
	CHECK(MS_READY == nav_mission_state());
	CHECK(0 == g_cal_req_ms);
	CHECK(NULL == strstr(g_log, "hands off"));

	reset();
	run_boot();
	CHECK(MS_READY == nav_mission_state());
	CHECK(0 == g_cal_req_ms);
	CHECK(NULL == strstr(g_log, "hands off"));
	CHECK(log_before("CALIBRATING -> READY", "GP20 = start/resume/reset"));
}

/* text appears in the log exactly once */
static bool log_once(const char *text)
{
	const char *p_at = strstr(g_log, text);

	return (NULL != p_at) && (NULL == strstr(p_at + 1, text));
}

/* N6: an MQTT command sent while calibrating is dropped, not applied the
   moment Ready is reached (the car drove off with nobody at GP20). */
static void cmd_during_cal_dropped(void)
{
	/* boot: a START sent during the boot calibration */
	reset();
	g_mq_during_cal = (UB)MCMD_START;
	run_boot();
	CHECK(MS_READY == nav_mission_state());
	CHECK(0 == g_mq_n);
	CHECK(0U == (g_flg[g_nav_flgid] & NAV_FLG_MISSION_CMD));
	CHECK(log_before("MQTT command 1 sent while CALIBRATING: dropped",
			 "mission CALIBRATING -> READY"));
	on_mission_cmd();			/* the main loop: nothing left */
	CHECK(MS_READY == nav_mission_state());

	/* GP20 fault reset, START sent during the hands-off wait/calibration */
	reset();
	g_mission = (uint32_t)MS_FAULT;
	g_pins_low = 1U << BTN_START_PIN;
	g_mq_during_cal = (UB)MCMD_START;
	on_button();
	CHECK(MS_READY == nav_mission_state());
	CHECK(0 == g_mq_n);
	CHECK(log_once("sent while CALIBRATING: dropped"));

	/* MQTT RESET from Fault: the START sent mid-calibration is not read
	   by the same loop once Ready is reached */
	reset();
	g_mission = (uint32_t)MS_FAULT;
	mq_put((UB)MCMD_RESET);
	g_mq_during_cal = (UB)MCMD_START;
	on_mission_cmd();
	CHECK(MS_READY == nav_mission_state());
	CHECK(NULL == strstr(g_log, "READY -> RUNNING"));
	CHECK(log_once("MQTT command 1 sent while CALIBRATING: dropped"));

	/* calibration fails: a RESET sent meanwhile does not start another */
	reset();
	g_mission = (uint32_t)MS_FAULT;
	g_cal_fail = 1;
	mq_put((UB)MCMD_RESET);
	g_mq_during_cal = (UB)MCMD_RESET;
	on_mission_cmd();
	CHECK(MS_FAULT == nav_mission_state());
	CHECK(log_once("mission FAULT -> CALIBRATING"));
	CHECK(log_once("MQTT command 4 sent while CALIBRATING: dropped"));

	/* a START sent in Ready still starts the run */
	reset();
	g_mission = (uint32_t)MS_READY;
	mq_put((UB)MCMD_START);
	on_mission_cmd();
	CHECK(MS_RUNNING == nav_mission_state());
	CHECK(NULL == strstr(g_log, "dropped"));
}

int main(void)
{
	refused_keeps_command();
	done_uses_command();
	newer_command_waits();
	edges();
	fault_reset_waits();
	reset_and_boot_no_wait();
	cmd_during_cal_dropped();
	printf("nav junctions and calibration: %d checks, %d failures\n", g_checks, g_failures);
	return (0 == g_failures) ? 0 : 1;
}
