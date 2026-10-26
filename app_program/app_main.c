/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	app_main.c
 *	usermain() for the car on the micro T-Kernel 3.0 RP2040 SMP port.
 *
 *	Creates and starts the application tasks, prints the boot banner, then
 *	sleeps.  The kernel-port demo pipeline (producer/consumer/monitor) and
 *	the GP21->GP10 led_test were removed on 2026-10-21: led_test drove the
 *	right motor input, and the demo held a mutex and a message buffer the
 *	car needs.  Both are in the git history.
 *
 *	Tasks started here:
 *	    wifi_status_task  Wi-Fi/MQTT console lines   (WIFI=cyw43, TM_WIFI_REPORT=1;
 *	                      development only, off for demo day)
 *	    comms_task        MQTT telemetry, Buddy 1    (WIFI_MQTT=1)
 *	    motion_task       motors and encoders, Buddy 2 (always; core 1,
 *	                      plus its motion_report printer on core 0)
 *	    blink_task        GP16 liveness LED          (TM_BLINK_LED=1 only)
 *	    race_test tasks   Tribunal 1 race test.       (TM_RACE_TEST=1 only)
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "usb_console_compat.h"
#include "demo_tasks.h"
#include "race_test/race_test.h"
#include "comms_task/comms_task.h"
#include "obstacle_task/obstacle_task.h" // buddy 5 obstacle scan task
#include "obstacle_task/scan_if.h"      // buddy 5: scan_task links
#include "nav_task/nav_ctrl.h"          // buddy 5: nav_ctrl_task (mission state)
#include "motor_task/motor_task.h"      // buddy 2: motion_task (motors, encoders)
#include "imu_task/imu_task.h" // buddy4: imu_task
#if TM_WIFI_CYW43
/* Plain C types on purpose - see the note in cyw43_utk.h. */
#include "cyw43_utk.h"
#endif

/* ------------------------------------------------------------------ *
 *  Tunables
 * ------------------------------------------------------------------ */

/* Every task uses 4096 bytes.  The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ	4096

/* Wi-Fi status reporter (below): a development console, not part of the car
   design.  It came with the kernel port and prints every [wifi]/[mqtt] line
   the F1 and MQTT evidence relies on.  Set to 0 for the demo-day build
   (docs/comms/TASK_LAYOUT_RECONCILIATION.md, section 4 decision 8). */
#ifndef TM_WIFI_REPORT
#define TM_WIFI_REPORT	1
#endif

#if TM_WIFI_CYW43 && TM_WIFI_REPORT
/* ------------------------------------------------------------------ *
 *  WiFi status -- only built when WIFI=cyw43 and TM_WIFI_REPORT=1
 *
 *  The radio is not driven from here.  A dedicated service task owns the
 *  CYW43439, lwIP and the echo session -- pinned to processor 1 under SMP,
 *  and simply the only processor there is on a single-core build.  This task
 *  only reads the snapshot that service publishes and prints what changed.
 *  Reading is safe from any task: cyw43_utk_get_status() takes a seqlock
 *  copy, which is why the application goes through it rather than calling
 *  lwip_utk_udp_get_status() directly.
 * ------------------------------------------------------------------ */

#define WIFI_IDLE_MS	3000	/* nothing happening: report rarely      */
#define WIFI_BUSY_MS	 200	/* session running: drain the echo ring  */

/* Compose, then emit.  tm_printf is atomic per call, but a line built from
   several calls can still be spliced by a higher-priority task printing
   between them -- so every line below is formatted into a local buffer and
   emitted with exactly one call. */
LOCAL void fmt_ipv4(UB *dst, const UB *a)
{
	tm_sprintf(dst, (UB *)"%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
}

/* Names for the connection state machine's log (cyw43_utk.h). */
LOCAL const char *net_state_name(UW state)
{
	static const char *const name[] = {
		"SCANNING", "JOINING", "WAIT_LEASE", "ONLINE", "BACKOFF"
	};

	return (state < sizeof(name) / sizeof(name[0])) ? name[state] : "?";
}

#if TM_WIFI_MQTT
/* Names for the MQTT session state (CYW43_MQTT_* in cyw43_utk.h). */
LOCAL const char *mqtt_state_name(UW state)
{
	static const char *const name[] = {
		"WAIT_ADDR", "CONNECTING", "CONNECTED", "BACKOFF", "FAILED"
	};

	return (state < sizeof(name) / sizeof(name[0])) ? name[state] : "?";
}
#endif

/* " why=..." for a log entry, or "" on the normal forward path.  The link
   codes are CYW43_LINK_FAIL / _NONET / _BADAUTH from cyw43.h, which this file
   cannot include. */
LOCAL void fmt_net_why(UB *dst, const T_CYW43_UTK_NET_EVENT *ev)
{
	const char	*link;

	switch(ev->why) {
	case CYW43_NET_WHY_JOIN_REQUEST:
		tm_sprintf(dst, (UB *)" why=join-request(%d)", ev->code);
		break;
	case CYW43_NET_WHY_JOIN_FAILED:
		link = (ev->code == -1) ? "FAIL"
		     : (ev->code == -2) ? "NONET"
		     : (ev->code == -3) ? "BADAUTH" : "?";
		tm_sprintf(dst, (UB *)" why=join-failed(%s)", link);
		break;
	case CYW43_NET_WHY_JOIN_TIMEOUT:
		tm_sprintf(dst, (UB *)" why=join-timeout");
		break;
	case CYW43_NET_WHY_LEASE_TIMEOUT:
		tm_sprintf(dst, (UB *)" why=lease-timeout");
		break;
	case CYW43_NET_WHY_LINK_LOST:
		tm_sprintf(dst, (UB *)" why=link-lost");
		break;
	case CYW43_NET_WHY_LEASE_LOST:
		tm_sprintf(dst, (UB *)" why=lease-lost");
		break;
	default:
		dst[0] = '\0';
		break;
	}
}

LOCAL void wifi_status_task(INT stacd, void *exinf)
{
	/* static: the snapshot is ~2.7 KB, too much of a 4 KB stack.  Only one
	   instance of this task ever runs. */
	static T_CYW43_UTK_STATUS	st;
	UW			net_cursor = 0;	/* state changes printed so far */
	UW			last_link = 0xffffffffU;
	UW			cursor = 0;	/* echoes printed so far */
	UB			ip[16];
	UW			last_dhcp = 0xffffffffU;
#if TM_WIFI_MQTT
	UW			last_mqtt = 0xffffffffU;
#endif
	UW			stalled_polls = 0;
	UW			shown_session = 0xffffffffU;
	UW			summarised = 0xffffffffU;
	INT			i;

	(void)stacd; (void)exinf;

	/* Same enumeration wait usermain() uses before its banner.  Without it
	   this task races ahead of the banner and its first lines are lost
	   before the host is reading. */
	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(700);		/* let usermain() print the banner first */

	while(1) {
		cyw43_utk_get_status(&st);

		/* Connection state changes.  The radio task logs every transition,
		   so none is lost between these polls even though JOINING lasts
		   less than WIFI_IDLE_MS.  join# / lease# / lost are the totals at
		   the time of the change, t is ms since boot. */
		if(st.net_log_seq - net_cursor > CYW43_NET_LOG) {
			tm_printf((UB *)"[wifi] (%u state changes not shown)\n",
				  st.net_log_seq - net_cursor - CYW43_NET_LOG);
			net_cursor = st.net_log_seq - CYW43_NET_LOG;
		}
		while(net_cursor != st.net_log_seq) {
			const T_CYW43_UTK_NET_EVENT *ev =
				&st.net_log[net_cursor % CYW43_NET_LOG];
			UB why[32];

			fmt_net_why(why, ev);
			tm_printf((UB *)"[wifi] state=%s%s join#=%u lease#=%u"
				       " lost=%u t=%ums\n",
				  net_state_name(ev->state), why,
				  ev->join_attempts, ev->lease_attempts,
				  ev->link_losses, ev->t_ms);
			net_cursor++;
		}

		/* Link transitions, printed once each rather than every tick. */
		if(st.link_up != last_link) {
			last_link = st.link_up;
			if(st.link_up) {
				tm_printf((UB *)"[wifi] link UP  rssi=%d  mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
					  st.link_rssi, st.mac[0], st.mac[1],
					  st.mac[2], st.mac[3], st.mac[4], st.mac[5]);
				/* The address is reported by its own transition
				   below -- DHCP usually finishes after this. */
			} else {
				tm_printf((UB *)"[wifi] link DOWN\n");
			}
		}

		/* DHCP completes some time after the link comes up, so report it
		   on its own transition.  Without this the address is only ever
		   printed in the rare case it lands in the same poll as link-up,
		   and a run that never gets an address looks like silence. */
		if(st.dhcp_complete != last_dhcp) {
			last_dhcp = st.dhcp_complete;
			if(st.dhcp_complete) {
				UB gw[16], dns[16];
				fmt_ipv4(ip, st.dhcp_address);
				fmt_ipv4(gw, st.dhcp_gateway);
				fmt_ipv4(dns, st.dhcp_dns);
				tm_printf((UB *)"[wifi] ip=%s  gw=%s  dns=%s\n",
					  ip, gw, dns);
			}
		}

#if TM_WIFI_MQTT
		/* One line per session state change.  Changes faster than this
		   task polls are collapsed, but the counters still show them:
		   conn# counts CONNACKs and lost counts sessions that ended. */
		if(st.mqtt_state_changes != last_mqtt) {
			UB broker[16];
			last_mqtt = st.mqtt_state_changes;
			fmt_ipv4(broker, st.mqtt_broker);
			tm_printf((UB *)"[mqtt] state=%s  %s:%u  conn#=%u lost=%u"
				       "  pub=%u err=%u drop=%u  last=%d/%d\n",
				  mqtt_state_name(st.mqtt_state), broker,
				  st.mqtt_port, st.mqtt_connects,
				  st.mqtt_sessions_lost, st.mqtt_published,
				  st.mqtt_publish_errors, st.mqtt_queue_dropped,
				  st.mqtt_connect_request, st.mqtt_connection_result);
		}
#endif

		if(!st.udp_enabled) {
#if TM_WIFI_DHCP
			/* Diagnostic (F1.0): linked but no lease yet.  polls= frozen
			   between lines means the radio task has stopped, and stage=
			   says where; rd= climbing means it stopped mid-publish.  The
			   address fields are what lwIP holds so far.  (The UDP profile
			   has its own "[udp] waiting" line.) */
			if(st.link_up && !st.dhcp_complete) {
				UB gw[16], dns[16];
				fmt_ipv4(ip, st.dhcp_address);
				fmt_ipv4(gw, st.dhcp_gateway);
				fmt_ipv4(dns, st.dhcp_dns);
				tm_printf((UB *)"[wifi] waiting for lease: state=%u tries=%u"
					       " t=%ums addr=%s gw=%s dns=%s polls=%u"
					       " stage=%u rd=%u\n",
					  st.dhcp_state, st.dhcp_tries,
					  st.dhcp_elapsed_ms, ip, gw, dns,
					  st.poll_count, st.poll_stage,
					  st.status_retries);
			}
#endif
			tk_dly_tsk(WIFI_IDLE_MS);
			continue;
		}

		/* Nothing sent yet?  Say what is still missing rather than going
		   quiet.  The session cannot start until the interface is up and
		   has an address. */
		if(st.udp_packets_sent == 0 && st.udp_write_seq == 0) {
			if(++stalled_polls == 25U) {	/* ~5 s at the busy rate */
				tm_printf((UB *)"[udp] waiting: link=%u netif_up=%u"
					       " link_up=%u dhcp=%u addr=%u.%u.%u.%u\n",
					  st.link_up, st.netif_up, st.netif_link_up,
					  st.dhcp_complete, st.dhcp_address[0],
					  st.dhcp_address[1], st.dhcp_address[2],
					  st.dhcp_address[3]);
				stalled_polls = 0;
			}
		} else {
			stalled_polls = 0;
		}

		/* Announce the session once, when the first packet goes out.
		   Before that the target is still 0.0.0.0:0 and printing it is
		   noise, not information. */
		if(st.udp_packets_sent != 0 && st.udp_session_count != shown_session) {
			shown_session = st.udp_session_count;
			fmt_ipv4(ip, st.udp_target);
			tm_printf((UB *)"[udp] session %u: echoing to %s:%u,"
				       " %u packets of %u bytes\n",
				  st.udp_session_count + 1U, ip, st.udp_port,
				  st.udp_expected_packets, st.udp_payload_size);
		}

		/* Drain the ring: print every echo that arrived since last look.
		   If the reader ever falls more than LWIP_UTK_UDP_RING behind,
		   skip forward and say so rather than printing stale slots. */
		if(st.udp_write_seq > cursor) {
			if(st.udp_write_seq - cursor > CYW43_UDP_RING) {
				tm_printf((UB *)"[udp] (%u echoes not shown, reader behind)\n",
					  st.udp_write_seq - cursor - CYW43_UDP_RING);
				cursor = st.udp_write_seq - CYW43_UDP_RING;
			}
			while(cursor < st.udp_write_seq) {
				UW	slot = cursor % CYW43_UDP_RING;
				UB	line[96];
				INT	n;

				n = tm_sprintf(line, (UB *)"[udp] echo #%u, %u bytes:",
					       st.udp_ring_seq[slot],
					       st.udp_ring_len[slot]);
				for(i = 0; i < 12 && i < (INT)st.udp_ring_len[slot]; i++) {
					n += tm_sprintf(&line[n], (UB *)" %02x",
							st.udp_ring_payload[slot][i]);
				}
				if(st.udp_ring_len[slot] > 12) {
					n += tm_sprintf(&line[n], (UB *)" ...");
				}
				tm_printf((UB *)"%s\n", line);
				cursor++;
			}
		}

		if(st.udp_complete && st.udp_session_count != summarised) {
			summarised = st.udp_session_count;
			fmt_ipv4(ip, st.udp_target);
			tm_printf((UB *)"[udp] %s:%u  sent=%u recv=%u matched=%u/%u"
				       "  corrupt=%u retries=%u errs=%u  %u ms\n",
				  ip, st.udp_port, st.udp_packets_sent,
				  st.udp_packets_received, st.udp_packets_matched,
				  st.udp_expected_packets, st.udp_corrupt,
				  st.udp_retries, st.udp_send_errors,
				  st.udp_elapsed_ms);
			tm_printf((UB *)"[udp] session %u complete: result=%d%s\n",
				  st.udp_session_count + 1U, st.udp_result,
				  (st.udp_packets_matched == st.udp_expected_packets
				   && st.udp_corrupt == 0)
				  ? "  ALL PACKETS ECHOED" : "");
		}

		/* Poll fast whenever a session could be running, not only once
		   one has been observed running.  A session lasts about 1.4 s
		   and repeats every 5 s; at the idle rate this task would sleep
		   straight through one and find the ring already wrapped.  Once
		   the link is up and UDP is enabled, stay at the fast rate. */
		tk_dly_tsk((st.udp_enabled && st.link_up)
			   ? WIFI_BUSY_MS : WIFI_IDLE_MS);
	}
}

/* Priority 17: below comms_task (16), so a console printer never delays
   telemetry.  Core 0 with the radio, never on the control core (core 1). */
LOCAL T_CTSK ctsk_wifi = {
	.itskpri	= 17,
	.stksz		= STACK_SZ,
	.task		= wifi_status_task,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC1,
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};
#endif	/* TM_WIFI_CYW43 && TM_WIFI_REPORT */

/* ------------------------------------------------------------------ *
 *  Task definitions
 * ------------------------------------------------------------------ */

#if TM_BLINK_LED
LOCAL T_CTSK ctsk_blink = {
	.itskpri	= 10,
	.stksz		= STACK_SZ,
	.task		= blink_task,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC1,
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};
#endif

/* buddy4: imu_task: reads the IMU, detects humps and abnormal motion, and raises
 * STOP_IMU / IMU_FAULT (Buddy 4). Week 6 task table: priority 12, core 1. */
LOCAL T_CTSK ctsk_imu = {
	.itskpri	= 12,			/* Lower number = higher priority. Below
						   ir_sense_task (10) and motion_task (11),
						   above scan_task (13). Priorities only
						   order tasks within one core. */
	.stksz		= STACK_SZ,		/* Stack in bytes. Reuses master's shared
						   constant; size it from the measured stack
						   high-water mark later. */
	.task		= imu_task,		/* Entry function, declared in imu_task.h. */
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG		/* Task is written in C. */
			| TA_RNG3		/* Runs at protection level 3 (user task). */
			| TA_ASSPRC,		/* Pin to a specific core. Unpinned tasks
						   can land on core 0 and be preempted by
						   the radio task. */
	.assprc		= TP_PRC2,		/* Processor 2 = RP2040 core 1, where the
						   control tasks and encoder/echo ISRs run.
						   (TP_PRC1 is core 0: radio and console.) */
#else
	.tskatr		= TA_HLNG | TA_RNG3,	/* Single-core build: there is no second
						   core, so no pinning. */
#endif
};

// Buddy 5 obstacle scan task (scan_task in the design): priority 13, core 1
LOCAL T_CTSK ctsk_obstacle = {
	.itskpri	= 13,
	.stksz		= STACK_SZ,
	.task		= obstacle_task,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC2,	/* processor 2 = RP2040 core 1 */
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};

// Buddy 5 nav_ctrl_task (mission state, buttons, buzzer): priority 15, core 0
LOCAL T_CTSK ctsk_nav = {
	.itskpri	= 15,
	.stksz		= STACK_SZ,
	.task		= nav_ctrl_task,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC1,	/* processor 1 = RP2040 core 0 */
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};


/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or say which one failed and stop.  A silent object
   failure here would surface much later as a mysterious hang. */
LOCAL BOOL made(const char *what, ID id)
{
	if(id <= E_OK) {
		tm_printf((UB *)"[init] tk_cre_%s failed, er=%d\n", what, id);
		return FALSE;
	}
	return TRUE;
}

/* Boot banner.  On a USB-CDC build anything printed before the host
   enumerates goes into a ring nobody reads, so wait for the link first.
   Bounded at ~20 s so a headless board still runs; on a UART build
   tm_usb_state() is a constant and this falls straight through. */
LOCAL void print_banner(void)
{
	INT	i;

	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(500);

	tm_printf((UB *)"\n=== uT-Kernel 3.0 / RP2040 car ===\n");
#if TK_SUPPORT_SMP
	tm_printf((UB *)"SMP build: %d processors\n\n", TK_MAX_CORE);
#else
	tm_printf((UB *)"single-core build\n\n");
#endif
}

EXPORT INT usermain(void)
{
	ID	tid;

	(void)tid;		/* unused when no task below is built in */

	/* Kernel objects that cross app tasks first, before any task starts
	   (design section 6.3).  tele_mbf: every task -> comms_task. */
	if(comms_tele_init() < E_OK) {
		tm_printf((UB *)"[init] comms_tele_init failed\n");
	}

	/* Buddy 5: nav_flg first, then tell scan_task where the mission state
	 * and nav_flg are. motion IDs stay 0 until Buddy 2 adds them (scan_task
	 * then simulates motion_task's replies). A failure here only prints:
	 * usermain() must not return (see the end of this function). */
	if(E_OK != nav_ctrl_init()) {
		tm_printf((UB *)"[init] nav_flg failed: nav/scan not started\n");
	} else {
		scan_if_bind_t scan_bind = {
			.nav_flgid		= nav_flag(),
			.p_mission_running	= nav_mission_running,
		};
		scan_if_bind(&scan_bind);

		tid = tk_cre_tsk(&ctsk_nav);      if(made("tsk(nav)",      tid)) tk_sta_tsk(tid, 0); // buddy 5 nav_ctrl_task
		tid = tk_cre_tsk(&ctsk_obstacle); if(made("tsk(obstacle)", tid)) tk_sta_tsk(tid, 0); // buddy 5 scan_task
	}

	/* Buddy 2: motion_task (priority 11, core 1) and motion_report
	   (priority 24, core 0), both created in motor_task.c with TA_ASSPRC.
	   Outside the nav block so the motors are braked even if nav_flg
	   failed.  Motion IDs are not bound yet: scan_task still simulates
	   motion's replies. */
	{
		ER er = motor_task_start();
		if(er < E_OK) tm_printf((UB *)"[init] motor_task_start failed, er=%d\n", er);
	}

	/* Buddy 4: imu_task (priority 12, core 1). */
	tid = tk_cre_tsk(&ctsk_imu);      if(made("tsk(imu)",      tid)) tk_sta_tsk(tid, 0);

#if TM_BLINK_LED
	tid = tk_cre_tsk(&ctsk_blink);    if(made("tsk(blink)",    tid)) tk_sta_tsk(tid, 0);
#endif
#if TM_WIFI_CYW43 && TM_WIFI_REPORT
	tid = tk_cre_tsk(&ctsk_wifi);     if(made("tsk(wifi)",     tid)) tk_sta_tsk(tid, 0);
#endif
#if TM_WIFI_MQTT
	{
		ER er = comms_task_start();
		if(er < E_OK) tm_printf((UB *)"[init] comms_task_start failed, er=%d\n", er);
	}
#endif

#if TM_RACE_TEST
	/* Two-core demonstration of the packed-word hazard.  Off by default:
	   flip TM_RACE_TEST in race_test.h to build it in. */
	if(race_test_start() < E_OK) {
		tm_printf((UB *)"[init] race_test_start failed\n");
	}
#endif

	print_banner();

	/* The initial task has nothing left to do.  It must not return: on
	   return the kernel shuts the system down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}