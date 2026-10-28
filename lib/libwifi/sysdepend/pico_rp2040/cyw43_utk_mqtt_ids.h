/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CYW43_UTK_MQTT_IDS_H
#define CYW43_UTK_MQTT_IDS_H

/* Constants only, no prototypes: included by cyw43_utk.h and by
   cyw43_utk_kernel.c, which cannot include cyw43_utk.h because its
   int32_t prototypes clash with the kernel's INT/ER ones. */

/* Mirrors LWIP_UTK_MQTT_TOPIC_* and LWIP_UTK_MQTT_* (session state), checked
   in cyw43_utk.c.  The topic is the first argument of cyw43_utk_mqtt_send(). */
#define CYW43_MQTT_TOPIC_TELEMETRY  0
#define CYW43_MQTT_TOPIC_HEARTBEAT  1
#define CYW43_MQTT_TOPIC_STATE      2

#define CYW43_MQTT_WAIT_ADDR    0
#define CYW43_MQTT_CONNECTING   1
#define CYW43_MQTT_CONNECTED    2
#define CYW43_MQTT_BACKOFF      3
#define CYW43_MQTT_FAILED       4

/* Outbound record queue (cyw43_utk_kernel.c).  Packed records only: the
   largest is the 23-byte D snapshot (docs/comms/MQTT_SCHEMA.md section 7);
   24 leaves room for one more byte.  The buffer is sized for QUEUE_DEPTH
   records of RECORD_MAX (128 B);
   smaller records take less, so more of them fit -- 9 H records, measured
   on hardware 2026-10-21. */
#define CYW43_MQTT_RECORD_MAX   24
#define CYW43_MQTT_QUEUE_DEPTH  4

#endif
