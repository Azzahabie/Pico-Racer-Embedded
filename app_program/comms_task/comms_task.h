#ifndef COMMS_TASK_H
#define COMMS_TASK_H

#include <tk/tkernel.h>

/* comms_task: turns car state into MQTT records (docs/comms/README.md).
 *
 * How another task reports an event (MQTT_SCHEMA.md section 5):
 *
 *     UB body[COMM_W_INTAKE_SIZE_LINE];
 *     comms_tele_send(body, (INT)comm_w_pack_line(body, state, deviation));
 *
 * comms_tele_send() never blocks: it returns E_TMOUT and counts a drop when
 * the queue is full.  It works in every build; without MQTT nothing drains
 * the queue, so sends simply start dropping. */

/* The intake queue (tele_mbf).  Created by comms_tele_init(). */
IMPORT ID tele_mbfid;

/* Create tele_mbf, and in MQTT builds mission_cmd_mbf.  Called first in
   usermain(), before any task starts. */
IMPORT ER comms_tele_init(void);

/* mission_cmd_mbf for nav_bind_t.mission_cmd_mbfid (0 without MQTT). */
IMPORT ID comms_mission_mbf(void);

/* Queue one packed event body (from a comm_w_pack_* helper). */
IMPORT ER comms_tele_send(const UB *body, INT length);

/* Create and start comms_task.  MQTT builds (TM_WIFI_MQTT) only. */
IMPORT ER comms_task_start(void);

#endif /* COMMS_TASK_H */
