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

typedef struct {
    int32_t  init_result;        /* client alloc / broker address parse      */
    int32_t  connect_request;    /* what mqtt_client_connect() returned      */
    int32_t  connection_result;  /* mqtt_connection_status_t from the callback */
    int32_t  publish_result;     /* what mqtt_publish() returned             */
    int32_t  publish_ack;        /* err_t delivered to the publish callback  */
    int32_t  result;             /* overall verdict, valid once complete     */
    uint32_t initialized;
    uint32_t started;
    uint32_t connected;
    uint32_t published;
    uint32_t complete;
    uint32_t connect_attempts;
    uint32_t publish_attempts;
    uint32_t poll_calls;
    uint32_t guard_blocked;
    uint32_t connect_ms;
    uint32_t elapsed_ms;
    uint8_t  broker[4];
    uint16_t port;
    char     topic[LWIP_UTK_MQTT_TOPIC_MAX];
    char     payload[LWIP_UTK_MQTT_PAYLOAD_MAX];
} T_LWIP_UTK_MQTT_STATUS;

void lwip_utk_mqtt_poll(struct netif *netif);
void lwip_utk_mqtt_get_status(T_LWIP_UTK_MQTT_STATUS *status);

#endif