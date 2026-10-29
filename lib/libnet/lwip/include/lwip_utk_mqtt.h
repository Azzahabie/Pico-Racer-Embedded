/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LWIP_UTK_MQTT_H
#define LWIP_UTK_MQTT_H

#include <stdint.h>

/* Plain C types only: this header is pulled into code that cannot see the
 * lwIP headers, so it must not depend on them. */
struct netif;

#define LWIP_UTK_MQTT_TOPIC_MAX    48U
#define LWIP_UTK_MQTT_PAYLOAD_MAX  48U

/* Outbound topics, by index (docs/comms/MQTT_SCHEMA.md section 2).  The full
 * names are built from UTK_MQTT_CLIENT_ID in lwip_utk_mqtt.c, so callers pass
 * an index rather than a string. */
#define LWIP_UTK_MQTT_TOPIC_TELEMETRY  0U
#define LWIP_UTK_MQTT_TOPIC_HEARTBEAT  1U
#define LWIP_UTK_MQTT_TOPIC_STATE      2U
#define LWIP_UTK_MQTT_TOPIC_COUNT      3U

/* Inbound topics, subscribed at QoS 1 on every new session (MQTT_SCHEMA.md
 * section 8; design decision 22).  A message longer than
 * LWIP_UTK_MQTT_IN_MAX bytes is rejected here, never truncated. */
#define LWIP_UTK_MQTT_IN_CMD   0U
#define LWIP_UTK_MQTT_IN_MAP   1U
#define LWIP_UTK_MQTT_IN_COUNT 2U
#define LWIP_UTK_MQTT_IN_MAX   7U

/* Called inside the radio task with one complete inbound message; the
 * pointer is valid only during the call.  Returns 0 when it was kept. */
typedef int32_t (*lwip_utk_mqtt_inbound_fn)(uint32_t topic,
                                            const uint8_t *payload,
                                            uint32_t length);

/* Session state machine (docs/comms/WIFI_SUBSYSTEM_DESIGN.md section 6.2). */
#define LWIP_UTK_MQTT_WAIT_ADDR   0U   /* no address yet, or it was lost      */
#define LWIP_UTK_MQTT_CONNECTING  1U   /* connect sent, waiting for CONNACK   */
#define LWIP_UTK_MQTT_CONNECTED   2U
#define LWIP_UTK_MQTT_BACKOFF     3U   /* session failed or lost; retry later */
#define LWIP_UTK_MQTT_FAILED      4U   /* bad broker config or no memory; terminal */

typedef struct {
    int32_t  init_result;        /* client alloc / broker address parse        */
    int32_t  connect_request;    /* last mqtt_client_connect() return          */
    int32_t  connection_result;  /* last mqtt_connection_status_t from the callback */
    int32_t  publish_result;     /* last mqtt_publish() return                 */
    int32_t  publish_ack;        /* last err_t delivered to the publish callback */
    uint32_t state;              /* LWIP_UTK_MQTT_*                            */
    uint32_t state_changes;      /* bumps on every transition                  */
    uint32_t connect_attempts;
    uint32_t connects;           /* CONNACKs accepted                          */
    uint32_t sessions_lost;      /* connected sessions that ended              */
    uint32_t publish_attempts;
    uint32_t published;          /* publishes handed to TCP (QoS 0 callback OK) */
    uint32_t publish_errors;
    uint32_t poll_calls;
    uint32_t guard_blocked;
    uint32_t connect_ms;         /* last connect request to CONNACK            */
    uint32_t backoff_ms;         /* wait before the next connect attempt       */
    int32_t  subscribe_result;   /* last mqtt_subscribe() return or SUBACK err */
    uint32_t subscribed;         /* bit per LWIP_UTK_MQTT_IN_*, this session   */
    uint32_t inbound_received;   /* messages handed to the inbound function   */
    uint32_t inbound_rejected;   /* unknown topic, empty or too long           */
    uint32_t inbound_dropped;    /* the inbound function refused (queue full)  */
    uint8_t  broker[4];
    uint16_t port;
    uint8_t  payload_len;        /* length of the last publish                 */
    char     topic[LWIP_UTK_MQTT_TOPIC_MAX];       /* last publish's topic   */
    uint8_t  payload[LWIP_UTK_MQTT_PAYLOAD_MAX];   /* last publish, binary   */
} T_LWIP_UTK_MQTT_STATUS;

/* Radio-owner task only: every call below touches lwIP. */
void lwip_utk_mqtt_poll(struct netif *netif);
int  lwip_utk_mqtt_ready(void);
int32_t lwip_utk_mqtt_offer(uint32_t topic, const uint8_t *payload,
                            uint32_t length);
void lwip_utk_mqtt_get_status(T_LWIP_UTK_MQTT_STATUS *status);

/* Any task: sets a flag that the next poll acts on (no lwIP call). */
void lwip_utk_mqtt_request_restart(void);

/* Radio-owner task, before the first poll: where inbound messages go. */
void lwip_utk_mqtt_set_inbound(lwip_utk_mqtt_inbound_fn fn);

#endif
