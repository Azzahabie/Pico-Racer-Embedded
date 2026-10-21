#ifndef COMMS_TASK_H
#define COMMS_TASK_H

#include <tk/tkernel.h>

/* comms_task: turns car state into MQTT records (docs/comms/README.md).
 * So far it publishes only the H heartbeat.  Created and started from
 * usermain() in MQTT builds (TM_WIFI_MQTT). */
IMPORT ER comms_task_start(void);

#endif /* COMMS_TASK_H */
