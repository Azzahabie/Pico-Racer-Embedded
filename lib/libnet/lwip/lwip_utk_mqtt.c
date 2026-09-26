/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* MQTT publish qualification for the NO_SYS lwIP owner task.
 *
 * One client, one topic, one message: connect, publish at QoS 0, and record
 * what happened.  The subscription/command side and the telemetry envelope are
 * later deliverables; this module exists to prove the MQTT path end to end
 * against a broker we control, and to publish its own state so the console can
 * show it rather than leaving the verdict on the host alone.
 *
 * Like every other lwIP client here it is driven from a single poll entry
 * point, called only by the radio-owner task: lwIP is NO_SYS, so its callbacks
 * and timers only ever run inside that task's loop.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/apps/mqtt.h"

#include "lwip_utk_mqtt.h"
#include "broker_config.h"

#define UTK_MQTT_CONNECT_TIMEOUT_MS 15000U
#define UTK_MQTT_PUBLISH_TIMEOUT_MS 10000U
#define UTK_MQTT_SESSION_TIMEOUT_MS 45000U

/* Deliberately a test topic rather than a telemetry one: the topic schema and
 * payload format are their own deliverable and should not be pinned down by a
 * connectivity proof. */
#define UTK_MQTT_TOPIC   "robocar/test/mqtt"
#define UTK_MQTT_PAYLOAD "hello-from-pico"

_Static_assert(UTK_MQTT_BROKER_PORT > 0U && UTK_MQTT_BROKER_PORT <= 65535U,
               "UTK_MQTT_BROKER_PORT must be in the range 1..65535");

static T_LWIP_UTK_MQTT_STATUS mqtt_status = {
    .init_result = ERR_VAL,
    .connect_request = ERR_VAL,
    .connection_result = -1,
    .publish_result = ERR_VAL,
    .publish_ack = -1,
    .result = ERR_TIMEOUT,
    .port = (uint16_t)UTK_MQTT_BROKER_PORT,
};

static mqtt_client_t *mqtt_client;
static ip4_addr_t mqtt_broker;
static uint32_t started_ms;
static uint32_t connect_started_ms;
static uint32_t publish_started_ms;
static uint8_t publish_started;

static void finish(int32_t result)
{
    mqtt_status.result = result;
    mqtt_status.complete = 1;
}

static void copy_address(uint8_t dst[4], const ip4_addr_t *src)
{
    dst[0] = ip4_addr1(src);
    dst[1] = ip4_addr2(src);
    dst[2] = ip4_addr3(src);
    dst[3] = ip4_addr4(src);
}

static void copy_string(char *dst, size_t capacity, const char *src)
{
    size_t length = strlen(src);

    if(length >= capacity) length = capacity - 1U;
    memcpy(dst, src, length);
    dst[length] = '\0';
}

/* Both callbacks run inside the radio-owner task: lwIP only ever invokes them
 * from sys_check_timeouts() or from the netif input path, and both of those
 * run in the owner's poll loop. */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;

    mqtt_status.connection_result = (int32_t)status;
    if(status == MQTT_CONNECT_ACCEPTED) {
        mqtt_status.connected = 1;
        mqtt_status.connect_ms = sys_now() - connect_started_ms;
    } else {
        /* Refused or torn down: terminal for this one-shot phase. */
        finish(ERR_CONN);
    }
}

static void mqtt_publish_cb(void *arg, err_t result)
{
    (void)arg;

    mqtt_status.publish_ack = (int32_t)result;
    if(result == ERR_OK) {
        /* A QoS 0 publish is complete once the bytes are on the wire, so this
           fires from the TCP sent callback rather than from an ack. */
        mqtt_status.published = 1;
    } else {
        finish(result);
    }
}

static void initialize(struct netif *netif)
{
    const struct mqtt_connect_client_info_t client_info = {
        .client_id = UTK_MQTT_CLIENT_ID,
        .keep_alive = (u16_t)UTK_MQTT_KEEPALIVE_S,
    };
    err_t result;

    mqtt_status.initialized = 1;

    if(netif == NULL || !ip4addr_aton(UTK_MQTT_BROKER_ADDRESS, &mqtt_broker)
       || ip4_addr_isany_val(mqtt_broker)) {
        mqtt_status.init_result = ERR_ARG;
        finish(ERR_ARG);
        return;
    }
    copy_address(mqtt_status.broker, &mqtt_broker);
    copy_string(mqtt_status.topic, sizeof(mqtt_status.topic), UTK_MQTT_TOPIC);
    copy_string(mqtt_status.payload, sizeof(mqtt_status.payload),
                UTK_MQTT_PAYLOAD);

    mqtt_client = mqtt_client_new();
    if(mqtt_client == NULL) {
        mqtt_status.init_result = ERR_MEM;
        finish(ERR_MEM);
        return;
    }

    connect_started_ms = sys_now();
    mqtt_status.connect_attempts++;
    result = mqtt_client_connect(mqtt_client, (const ip_addr_t *)&mqtt_broker,
                                 (u16_t)UTK_MQTT_BROKER_PORT,
                                 mqtt_connection_cb, NULL, &client_info);
    mqtt_status.connect_request = result;
    mqtt_status.init_result = result;
    if(result != ERR_OK) finish(result);
}

void lwip_utk_mqtt_poll(struct netif *netif)
{
    uint32_t now = sys_now();
    err_t result;

    mqtt_status.poll_calls++;
    if(mqtt_status.complete) return;

    /* Wait for an address before connecting.  A connect issued without one
       fails, and arming the connect timeout first would measure it against a
       base that predates the interface coming up. */
    if(netif == NULL || !netif_is_up(netif) || !netif_is_link_up(netif)
       || ip4_addr_isany_val(*netif_ip4_addr(netif))) {
        mqtt_status.guard_blocked++;
        return;
    }

    if(!mqtt_status.initialized) {
        initialize(netif);
        if(mqtt_status.complete) return;
        started_ms = now;
        mqtt_status.started = 1;
    }

    mqtt_status.elapsed_ms = now - started_ms;
    if(mqtt_status.elapsed_ms >= UTK_MQTT_SESSION_TIMEOUT_MS) {
        finish(ERR_TIMEOUT);
        return;
    }

    if(!mqtt_status.connected) {
        if(now - connect_started_ms >= UTK_MQTT_CONNECT_TIMEOUT_MS) {
            finish(ERR_TIMEOUT);
        }
        return;
    }

    /* The connection callback already published; report and stop driving.
       lwIP's own cyclic timer keeps the session alive from here. */
    if(mqtt_status.published) {
        finish(ERR_OK);
        return;
    }

    if(!publish_started) {
        result = mqtt_publish(mqtt_client, UTK_MQTT_TOPIC, UTK_MQTT_PAYLOAD,
                              (u16_t)(sizeof(UTK_MQTT_PAYLOAD) - 1U),
                              0, 0, mqtt_publish_cb, NULL);
        mqtt_status.publish_result = result;
        mqtt_status.publish_attempts++;
        if(result == ERR_OK) {
            publish_started = 1;
            publish_started_ms = now;
        } else if(result != ERR_MEM) {   /* ERR_MEM simply retries next poll */
            finish(result);
        }
        return;
    }

    if(now - publish_started_ms >= UTK_MQTT_PUBLISH_TIMEOUT_MS) {
        finish(ERR_TIMEOUT);
    }
}

void lwip_utk_mqtt_get_status(T_LWIP_UTK_MQTT_STATUS *status)
{
    if(status != NULL) *status = mqtt_status;
}