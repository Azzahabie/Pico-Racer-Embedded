/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Persistent MQTT client for the NO_SYS lwIP owner task.
 *
 * One client, one broker connection for the whole run: wait for an address,
 * connect, publish whatever the radio task offers, and when the session dies
 * back off and connect again (docs/comms/WIFI_SUBSYSTEM_DESIGN.md sections
 * 6.2 and 10.1).  On every new session it subscribes to the command topics
 * (lwIP always connects with a clean session, so subscriptions do not
 * survive a reconnect) and hands each inbound message, copied, to the
 * function the radio task registered (design section 9.3).
 *
 * Like every other lwIP client here it is driven from a single poll entry
 * point, called only by the radio-owner task: lwIP is NO_SYS, so its callbacks
 * and timers only ever run inside that task's loop.  Other tasks never call in
 * here; they queue records through cyw43_utk_mqtt_send(), and the radio task
 * hands them over with lwip_utk_mqtt_offer().
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

/* Short on purpose.  A broker host that drops SYNs to a closed port (the
   Windows firewall does) never refuses, so each attempt runs to this timeout;
   at 15 s that swamped the backoff ladder and put NFR-09's 15 s recovery at
   risk (first hardware test, 2026-10-21).  A LAN CONNACK takes milliseconds. */
#define UTK_MQTT_CONNECT_TIMEOUT_MS  5000U
#define UTK_MQTT_BACKOFF_FIRST_MS    2000U
#define UTK_MQTT_BACKOFF_MAX_MS     15000U

#define UTK_MQTT_TOPIC_PREFIX "robocar/" UTK_MQTT_CLIENT_ID

_Static_assert(UTK_MQTT_BROKER_PORT > 0U && UTK_MQTT_BROKER_PORT <= 65535U,
               "UTK_MQTT_BROKER_PORT must be in the range 1..65535");
_Static_assert(sizeof(UTK_MQTT_TOPIC_PREFIX "/telemetry")
               <= LWIP_UTK_MQTT_TOPIC_MAX,
               "UTK_MQTT_CLIENT_ID is too long for LWIP_UTK_MQTT_TOPIC_MAX");

/* Indexed by LWIP_UTK_MQTT_TOPIC_*. */
static const char *const topic_name[LWIP_UTK_MQTT_TOPIC_COUNT] = {
    UTK_MQTT_TOPIC_PREFIX "/telemetry",
    UTK_MQTT_TOPIC_PREFIX "/heartbeat",
    UTK_MQTT_TOPIC_PREFIX "/state",
};

/* Indexed by LWIP_UTK_MQTT_IN_*. */
static const char *const in_topic_name[LWIP_UTK_MQTT_IN_COUNT] = {
    UTK_MQTT_TOPIC_PREFIX "/cmd",
    UTK_MQTT_TOPIC_PREFIX "/map",
};

static T_LWIP_UTK_MQTT_STATUS mqtt_status = {
    .init_result = ERR_VAL,
    .connect_request = ERR_VAL,
    .connection_result = -1,
    .publish_result = ERR_VAL,
    .publish_ack = -1,
    .state = LWIP_UTK_MQTT_WAIT_ADDR,
    .backoff_ms = UTK_MQTT_BACKOFF_FIRST_MS,
    .port = (uint16_t)UTK_MQTT_BROKER_PORT,
};

static mqtt_client_t *mqtt_client;
static ip4_addr_t mqtt_broker;
static uint32_t state_since_ms;
static uint8_t session_lost;     /* set by the connection callback */
/* Set by lwip_utk_mqtt_request_restart() from any task, taken by the poll. */
static volatile uint8_t restart_requested;

/* One record that mqtt_publish() refused with ERR_MEM, retried every poll. */
static uint8_t pending;
static uint32_t pending_topic;
static uint32_t pending_length;
static uint8_t pending_payload[LWIP_UTK_MQTT_PAYLOAD_MAX];

/* The inbound message being received.  lwIP's pointers die when its callback
   returns, so the bytes are copied here as they arrive. */
static lwip_utk_mqtt_inbound_fn inbound_fn;
static uint32_t in_topic;
static uint32_t in_length;
static uint8_t in_bad;
static uint8_t in_payload[LWIP_UTK_MQTT_IN_MAX];
static uint8_t sub_pending;      /* bit per LWIP_UTK_MQTT_IN_*: SUBACK awaited */

static void enter(uint32_t state, uint32_t now)
{
    mqtt_status.state = state;
    mqtt_status.state_changes++;
    state_since_ms = now;
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
 * run in the owner's poll loop.  They only record what happened; the state
 * machine in lwip_utk_mqtt_poll() acts on it. */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;

    mqtt_status.connection_result = (int32_t)status;
    if(status == MQTT_CONNECT_ACCEPTED) {
        mqtt_status.connects++;
        mqtt_status.connect_ms = sys_now() - state_since_ms;
    } else {
        /* Refused, timed out or torn down.  lwIP can report one refusal
           twice (CONNACK, then the close), so this is a flag, not a count. */
        session_lost = 1;
    }
}

static void mqtt_publish_cb(void *arg, err_t result)
{
    (void)arg;

    mqtt_status.publish_ack = (int32_t)result;
    if(result == ERR_OK) {
        /* A QoS 0 publish is complete once the bytes are on the wire, so this
           fires from the TCP sent callback rather than from an ack. */
        mqtt_status.published++;
    } else {
        mqtt_status.publish_errors++;
    }
}

/* Inbound: topic first, then the data in one or more pieces, the last one
   flagged.  Anything not on our two topics, empty, or longer than
   LWIP_UTK_MQTT_IN_MAX is counted and dropped whole. */
static void mqtt_inpub_cb(void *arg, const char *topic, u32_t tot_len)
{
    uint32_t i;

    (void)arg;
    in_length = 0;
    in_bad = 1;
    for(i = 0; i < LWIP_UTK_MQTT_IN_COUNT; i++) {
        if(strcmp(topic, in_topic_name[i]) == 0) {
            in_topic = i;
            in_bad = 0;
        }
    }
    if(tot_len == 0U || tot_len > LWIP_UTK_MQTT_IN_MAX) in_bad = 1;
}

static void mqtt_indata_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    (void)arg;
    if(!in_bad) {
        if(in_length + len > LWIP_UTK_MQTT_IN_MAX) {
            in_bad = 1;
        } else {
            memcpy(&in_payload[in_length], data, len);
            in_length += len;
        }
    }
    if((flags & MQTT_DATA_FLAG_LAST) == 0U) return;

    if(in_bad || in_length == 0U) {
        mqtt_status.inbound_rejected++;
    } else if(inbound_fn == NULL
              || inbound_fn(in_topic, in_payload, in_length) != 0) {
        mqtt_status.inbound_dropped++;
    } else {
        mqtt_status.inbound_received++;
    }
    in_bad = 1;     /* a stray data call without a new topic is rejected */
}

/* SUBACK.  arg carries the topic index. */
static void mqtt_sub_cb(void *arg, err_t result)
{
    uint32_t topic = (uint32_t)(uintptr_t)arg;

    sub_pending &= (uint8_t)~(1U << topic);
    mqtt_status.subscribe_result = (int32_t)result;
    if(result == ERR_OK) mqtt_status.subscribed |= 1U << topic;
}

/* Ask for each subscription not yet held or awaited.  ERR_MEM (no free
   request slot) is retried on the next poll. */
static void subscribe_missing(void)
{
    uint32_t i;
    err_t result;

    for(i = 0; i < LWIP_UTK_MQTT_IN_COUNT; i++) {
        if((mqtt_status.subscribed | sub_pending) & (1U << i)) continue;
        result = mqtt_subscribe(mqtt_client, in_topic_name[i], 1,
                                mqtt_sub_cb, (void *)(uintptr_t)i);
        mqtt_status.subscribe_result = (int32_t)result;
        if(result == ERR_OK) sub_pending |= (uint8_t)(1U << i);
    }
}

static void initialize(void)
{
    if(!ip4addr_aton(UTK_MQTT_BROKER_ADDRESS, &mqtt_broker)
       || ip4_addr_isany_val(mqtt_broker)) {
        mqtt_status.init_result = ERR_ARG;
        return;
    }
    copy_address(mqtt_status.broker, &mqtt_broker);

    mqtt_client = mqtt_client_new();
    mqtt_status.init_result = (mqtt_client == NULL) ? ERR_MEM : ERR_OK;
    if(mqtt_client != NULL) {
        mqtt_set_inpub_callback(mqtt_client, mqtt_inpub_cb, mqtt_indata_cb,
                                NULL);
    }
}

static void start_connect(uint32_t now)
{
    const struct mqtt_connect_client_info_t client_info = {
        .client_id = UTK_MQTT_CLIENT_ID,
        .keep_alive = (u16_t)UTK_MQTT_KEEPALIVE_S,
    };
    err_t result;

    session_lost = 0;
    enter(LWIP_UTK_MQTT_CONNECTING, now);
    mqtt_status.connect_attempts++;
    result = mqtt_client_connect(mqtt_client, (const ip_addr_t *)&mqtt_broker,
                                 (u16_t)UTK_MQTT_BROKER_PORT,
                                 mqtt_connection_cb, NULL, &client_info);
    mqtt_status.connect_request = result;
    if(result != ERR_OK) session_lost = 1;
}

/* Close whatever is left of the session and wait before trying again.  The
   backoff doubles up to its cap and resets on the next CONNACK. */
static void back_off(uint32_t now)
{
    if(mqtt_status.state == LWIP_UTK_MQTT_CONNECTED) mqtt_status.sessions_lost++;
    mqtt_disconnect(mqtt_client);   /* no-op if lwIP already closed it */
    pending = 0;
    enter(LWIP_UTK_MQTT_BACKOFF, now);
}

static void publish_pending(void)
{
    err_t result;

    mqtt_status.publish_attempts++;
    result = mqtt_publish(mqtt_client, topic_name[pending_topic],
                          pending_payload, (u16_t)pending_length,
                          0, (pending_topic == LWIP_UTK_MQTT_TOPIC_STATE) ? 1 : 0,
                          mqtt_publish_cb, NULL);   /* D on state is retained */
    mqtt_status.publish_result = result;
    if(result == ERR_MEM) return;   /* ring buffer full: retry next poll */

    pending = 0;
    if(result != ERR_OK) {
        mqtt_status.publish_errors++;
        return;
    }
    /* Diagnostic copy of what was sent.  Binary-safe: memcpy with the length,
       never a string function, because packed payloads contain zero bytes. */
    copy_string(mqtt_status.topic, sizeof(mqtt_status.topic),
                topic_name[pending_topic]);
    memcpy(mqtt_status.payload, pending_payload, pending_length);
    mqtt_status.payload_len = (uint8_t)pending_length;
}

void lwip_utk_mqtt_poll(struct netif *netif)
{
    uint32_t now = sys_now();
    int have_address;

    mqtt_status.poll_calls++;
    if(mqtt_status.state == LWIP_UTK_MQTT_FAILED) return;

    if(mqtt_client == NULL) {
        initialize();
        if(mqtt_status.init_result != ERR_OK) {
            enter(LWIP_UTK_MQTT_FAILED, now);
            return;
        }
    }

    have_address = netif != NULL && netif_is_up(netif) && netif_is_link_up(netif)
                   && !ip4_addr_isany_val(*netif_ip4_addr(netif));
    if(!have_address) {
        /* Wi-Fi dropped or the lease went away.  The session cannot survive
           that, so drop it now and start clean once an address returns. */
        mqtt_status.guard_blocked++;
        if(mqtt_status.state != LWIP_UTK_MQTT_WAIT_ADDR) {
            if(mqtt_status.state == LWIP_UTK_MQTT_CONNECTED) {
                mqtt_status.sessions_lost++;
            }
            mqtt_disconnect(mqtt_client);
            pending = 0;
            enter(LWIP_UTK_MQTT_WAIT_ADDR, now);
        }
        return;
    }

    /* A restart only means something to a CONNECTED session. */
    if(mqtt_status.state != LWIP_UTK_MQTT_CONNECTED) restart_requested = 0;

    switch(mqtt_status.state) {
    case LWIP_UTK_MQTT_WAIT_ADDR:
        start_connect(now);
        break;

    case LWIP_UTK_MQTT_CONNECTING:
        if(session_lost
           || now - state_since_ms >= UTK_MQTT_CONNECT_TIMEOUT_MS) {
            back_off(now);
        } else if(mqtt_client_is_connected(mqtt_client)) {
            mqtt_status.backoff_ms = UTK_MQTT_BACKOFF_FIRST_MS;
            mqtt_status.subscribed = 0;     /* clean session: none held */
            sub_pending = 0;
            enter(LWIP_UTK_MQTT_CONNECTED, now);
        }
        break;

    case LWIP_UTK_MQTT_CONNECTED:
        /* Do not trust a flag for liveness: ask the client every poll.  A
           restart request ends a session that still looks connected but
           has stopped moving (comms_task's stall rule): without it, a short
           AP outage waits out the 1.5 x keep-alive timeout. */
        if(session_lost || restart_requested
           || !mqtt_client_is_connected(mqtt_client)) {
            restart_requested = 0;
            back_off(now);
        } else {
            subscribe_missing();
            if(pending) publish_pending();
        }
        break;

    case LWIP_UTK_MQTT_BACKOFF:
        if(now - state_since_ms >= mqtt_status.backoff_ms) {
            mqtt_status.backoff_ms *= 2U;
            if(mqtt_status.backoff_ms > UTK_MQTT_BACKOFF_MAX_MS) {
                mqtt_status.backoff_ms = UTK_MQTT_BACKOFF_MAX_MS;
            }
            start_connect(now);
        }
        break;

    default:
        break;
    }
}

/* Any task: ask the next poll to drop a CONNECTED session and reconnect
   through the normal back-off.  Sets a flag only; never touches lwIP. */
void lwip_utk_mqtt_request_restart(void)
{
    restart_requested = 1;
}

void lwip_utk_mqtt_set_inbound(lwip_utk_mqtt_inbound_fn fn)
{
    inbound_fn = fn;
}

/* True when a record handed over now would be published immediately. */
int lwip_utk_mqtt_ready(void)
{
    return mqtt_status.state == LWIP_UTK_MQTT_CONNECTED && !pending;
}

/* Take one record and publish it, holding it for a retry if lwIP's output
   buffer is full.  Returns ERR_OK when the record was taken, ERR_CONN when not
   connected, ERR_INPROGRESS when the previous record is still held, ERR_VAL
   for a bad topic or length. */
int32_t lwip_utk_mqtt_offer(uint32_t topic, const uint8_t *payload,
                            uint32_t length)
{
    if(topic >= LWIP_UTK_MQTT_TOPIC_COUNT || payload == NULL || length == 0U
       || length > LWIP_UTK_MQTT_PAYLOAD_MAX) {
        return ERR_VAL;
    }
    if(mqtt_status.state != LWIP_UTK_MQTT_CONNECTED) return ERR_CONN;
    if(pending) return ERR_INPROGRESS;

    pending_topic = topic;
    pending_length = length;
    memcpy(pending_payload, payload, length);
    pending = 1;
    publish_pending();
    return ERR_OK;
}

void lwip_utk_mqtt_get_status(T_LWIP_UTK_MQTT_STATUS *status)
{
    if(status != NULL) *status = mqtt_status;
}
