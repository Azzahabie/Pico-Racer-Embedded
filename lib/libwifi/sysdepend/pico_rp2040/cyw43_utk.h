/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CYW43_UTK_H
#define CYW43_UTK_H

#include <stdint.h>

#define CYW43_UTK_DNS_TEST_HOSTNAME "example.com"

#define CYW43_UDP_RING     32
#define CYW43_UDP_PAYLOAD  48

/* Mirrors LWIP_UTK_MQTT_TOPIC_MAX / LWIP_UTK_MQTT_PAYLOAD_MAX in
   lib/libnet/lwip/include/lwip_utk_mqtt.h.  Duplicated for the same reason as
   the UDP geometry above -- this header is included by code that cannot see
   the lwIP headers -- and checked at compile time in cyw43_utk.c. */
#define CYW43_MQTT_TOPIC_MAX    48
#define CYW43_MQTT_PAYLOAD_MAX  48

/* Topic indices, session states and queue geometry. */
#include "cyw43_utk_mqtt_ids.h"

/* Connection state machine in cyw43_utk.c (net_state, and the state of each
   net_log entry). */
#define CYW43_NET_SCANNING      0   /* first scan still running */
#define CYW43_NET_JOINING       1
#define CYW43_NET_WAIT_LEASE    2
#define CYW43_NET_ONLINE        3
#define CYW43_NET_BACKOFF       4

/* Why a state was entered (net_log why); NONE for the normal forward path. */
#define CYW43_NET_WHY_NONE          0
#define CYW43_NET_WHY_JOIN_REQUEST  1   /* cyw43_wifi_join() refused; code = its result */
#define CYW43_NET_WHY_JOIN_FAILED   2   /* code = CYW43_LINK_FAIL / _NONET / _BADAUTH */
#define CYW43_NET_WHY_JOIN_TIMEOUT  3
#define CYW43_NET_WHY_LEASE_TIMEOUT 4
#define CYW43_NET_WHY_LINK_LOST     5   /* link dropped after association */
#define CYW43_NET_WHY_LEASE_LOST    6   /* lease gone while the link stayed up */

#define CYW43_NET_LOG   8

/* One state transition.  The counters are the totals at the moment of the
   transition, so a reader that drains the log late still prints them as they
   were. */
typedef struct {
    uint32_t t_ms;              /* since boot */
    uint32_t state;             /* CYW43_NET_* entered */
    uint32_t why;               /* CYW43_NET_WHY_* */
    int32_t  code;              /* driver result for JOIN_REQUEST / JOIN_FAILED */
    uint32_t join_attempts;
    uint32_t lease_attempts;
    uint32_t link_losses;
} T_CYW43_UTK_NET_EVENT;

typedef struct {
    volatile int32_t init_result;
    volatile uint32_t init_complete;
    volatile uint32_t ready;
    volatile uint32_t scan_active;
    volatile uint32_t scan_complete;
    volatile uint32_t scan_count;
    volatile int32_t strongest_rssi;
    volatile uint32_t poll_count;
    volatile uint32_t owner_prc;
    uint8_t mac[6];
    volatile uint32_t join_enabled;
    volatile uint32_t join_started;
    volatile uint32_t join_complete;
    volatile int32_t join_request_result;
    volatile int32_t join_result;
    volatile int32_t link_status;
    volatile uint32_t link_up;
    volatile uint32_t join_elapsed_ms;
    volatile int32_t link_rssi;
    uint8_t bssid[6];
    /* Connection state machine.  Counters are totals since boot. */
    volatile uint32_t net_state;        /* CYW43_NET_* */
    volatile uint32_t join_attempts;
    volatile uint32_t lease_attempts;   /* times WAIT_LEASE was entered */
    volatile uint32_t link_losses;
    /* Every transition, because the console polls slower than JOINING lasts
       and would otherwise miss states.  net_log_seq is the total ever written;
       a reader tracks its own cursor against it, as with the UDP ring. */
    T_CYW43_UTK_NET_EVENT net_log[CYW43_NET_LOG];
    uint32_t net_log_seq;
    volatile uint32_t lwip_initialized;
    volatile uint32_t netif_initialized;
    volatile uint32_t netif_up;
    volatile uint32_t netif_link_up;
    volatile uint32_t netif_default;
    volatile uint32_t netif_callbacks_ready;
    volatile uint32_t netif_no_sys;
    volatile uint32_t netif_mtu;
    uint8_t netif_mac[6];
    volatile uint32_t dhcp_enabled;
    volatile uint32_t dhcp_started;
    volatile uint32_t dhcp_complete;
    volatile int32_t dhcp_result;
    volatile uint32_t dhcp_elapsed_ms;
    volatile uint32_t dhcp_tries;
    volatile uint32_t dhcp_state;       /* lwIP dhcp_state_enum_t: 6 SELECTING,
                                           1 REQUESTING, 8 CHECKING, 10 BOUND */
    volatile uint32_t poll_stage;       /* last step the radio loop entered
                                           (see cyw43_utk_task); diagnostic */
    uint8_t dhcp_address[4];
    uint8_t dhcp_netmask[4];
    uint8_t dhcp_gateway[4];
    uint8_t dhcp_dns[4];
    volatile uint32_t dns_enabled;
    volatile uint32_t dns_started;
    volatile uint32_t dns_complete;
    volatile int32_t dns_request_result;
    volatile int32_t dns_result;
    volatile uint32_t dns_elapsed_ms;
    uint8_t dns_address[4];
    volatile uint32_t udp_enabled;
    volatile int32_t udp_init_result;
    volatile uint32_t udp_initialized;
    volatile uint32_t udp_started;
    volatile uint32_t udp_complete;
    volatile uint32_t udp_closed;
    volatile int32_t udp_send_result;
    volatile int32_t udp_result;
    volatile uint32_t udp_expected_packets;
    volatile uint32_t udp_payload_size;
    volatile uint32_t udp_packets_sent;
    volatile uint32_t udp_packets_received;
    volatile uint32_t udp_packets_matched;
    volatile uint32_t udp_retries;
    volatile uint32_t udp_send_errors;
    volatile uint32_t udp_corrupt;
    volatile uint32_t udp_unexpected;
    volatile uint32_t udp_elapsed_ms;
    uint8_t udp_target[4];
    uint16_t udp_port;
    /* Mirror of the UDP driver's echo ring, so the application can print
       every message rather than whichever was current when it last looked.
       udp_write_seq is the total ever written; a reader tracks its own
       cursor against it and prints what is new. */
    /* Geometry must match LWIP_UTK_UDP_RING / LWIP_UTK_UDP_PAYLOAD_SIZE;
       checked at compile time in cyw43_utk.c.  Readers must use
       CYW43_UDP_RING for their slot arithmetic -- a mismatched modulus reads
       the wrong half of the ring and silently duplicates entries. */
    uint8_t udp_ring_payload[CYW43_UDP_RING][CYW43_UDP_PAYLOAD];
    uint32_t udp_ring_seq[CYW43_UDP_RING];
    uint16_t udp_ring_len[CYW43_UDP_RING];
    uint32_t udp_write_seq;
    volatile uint32_t udp_session_count;
    volatile uint32_t tcp_enabled;
    volatile int32_t tcp_init_result;
    volatile int32_t tcp_connect_result;
    volatile int32_t tcp_send_result;
    volatile int32_t tcp_close_result;
    volatile int32_t tcp_result;
    volatile int32_t tcp_fatal_error;
    volatile uint32_t tcp_initialized;
    volatile uint32_t tcp_started;
    volatile uint32_t tcp_connected;
    volatile uint32_t tcp_complete;
    volatile uint32_t tcp_fin_sent;
    volatile uint32_t tcp_peer_fin;
    volatile uint32_t tcp_closed;
    volatile uint32_t tcp_aborted;
    volatile uint32_t tcp_expected_records;
    volatile uint32_t tcp_record_size;
    volatile uint32_t tcp_records_sent;
    volatile uint32_t tcp_records_matched;
    volatile uint32_t tcp_bytes_sent;
    volatile uint32_t tcp_bytes_acked;
    volatile uint32_t tcp_bytes_received;
    volatile uint32_t tcp_segments_received;
    volatile uint32_t tcp_short_segments;
    volatile uint32_t tcp_corrupt;
    volatile uint32_t tcp_send_errors;
    volatile uint32_t tcp_poll_calls;
    volatile uint32_t tcp_guard_blocked;
    volatile uint32_t tcp_start_delay_ms;
    volatile uint32_t tcp_connect_ms;
    volatile uint32_t tcp_close_ms;
    volatile uint32_t tcp_elapsed_ms;
    uint8_t tcp_target[4];
    uint16_t tcp_port;
    volatile uint32_t bulk_enabled;
    volatile int32_t bulk_init_result;
    volatile int32_t bulk_connect_result;
    volatile int32_t bulk_send_result;
    volatile int32_t bulk_close_result;
    volatile int32_t bulk_result;
    volatile int32_t bulk_fatal_error;
    volatile uint32_t bulk_initialized;
    volatile uint32_t bulk_started;
    volatile uint32_t bulk_connected;
    volatile uint32_t bulk_complete;
    volatile uint32_t bulk_fin_sent;
    volatile uint32_t bulk_peer_fin;
    volatile uint32_t bulk_closed;
    volatile uint32_t bulk_aborted;
    volatile uint32_t bulk_total_bytes;
    volatile uint32_t bulk_bytes_sent;
    volatile uint32_t bulk_bytes_acked;
    volatile uint32_t bulk_bytes_received;
    volatile uint32_t bulk_mismatches;
    volatile uint32_t bulk_first_mismatch;
    volatile uint32_t bulk_segments_received;
    volatile uint32_t bulk_max_segment;
    volatile uint32_t bulk_min_segment;
    volatile uint32_t bulk_split_boundaries;
    volatile uint32_t bulk_write_retries;
    volatile uint32_t bulk_connect_ms;
    volatile uint32_t bulk_close_ms;
    volatile uint32_t bulk_elapsed_ms;
    volatile uint32_t bulk_throughput_bps;
    uint8_t bulk_target[4];
    uint16_t bulk_port;
    /* MQTT client session.  Written only when WIFI_MQTT is on; the fields
       exist in every build so readers need no conditional. */
    volatile uint32_t mqtt_enabled;
    volatile int32_t  mqtt_init_result;
    volatile int32_t  mqtt_connect_request;
    volatile int32_t  mqtt_connection_result;
    volatile int32_t  mqtt_publish_result;
    volatile int32_t  mqtt_publish_ack;
    volatile uint32_t mqtt_state;          /* CYW43_MQTT_* */
    volatile uint32_t mqtt_state_changes;
    volatile uint32_t mqtt_connect_attempts;
    volatile uint32_t mqtt_connects;
    volatile uint32_t mqtt_sessions_lost;
    volatile uint32_t mqtt_publish_attempts;
    volatile uint32_t mqtt_published;
    volatile uint32_t mqtt_publish_errors;
    volatile uint32_t mqtt_poll_calls;
    volatile uint32_t mqtt_guard_blocked;
    volatile uint32_t mqtt_connect_ms;
    volatile uint32_t mqtt_backoff_ms;
    volatile uint32_t mqtt_queue_dropped;  /* sends refused: queue full / not ready */
    uint8_t mqtt_broker[4];
    uint16_t mqtt_port;
    uint8_t mqtt_payload_len;
    char mqtt_topic[CYW43_MQTT_TOPIC_MAX];        /* last publish */
    uint8_t mqtt_payload[CYW43_MQTT_PAYLOAD_MAX]; /* last publish, binary */
    volatile int32_t ipv4_config_result;
    volatile uint32_t ipv4_configured;
    volatile uint32_t arp_resolved;
    volatile uint32_t ping_started;
    volatile uint32_t ping_complete;
    volatile int32_t ping_send_result;
    volatile int32_t ping_result;
    volatile uint32_t ping_attempts;
    volatile uint32_t ping_elapsed_ms;
    uint8_t ipv4_address[4];
    uint8_t ipv4_netmask[4];
    uint8_t ipv4_gateway[4];
    uint8_t ping_target[4];
    uint8_t arp_mac[6];
    /* Stamped by the reader, not the radio task: how many torn snapshots the
     * seqlock in cyw43_utk_get_status() has had to discard since boot. */
    uint32_t status_retries;
} T_CYW43_UTK_STATUS;

int32_t cyw43_utk_start(void);
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status);
/* 1 while the MQTT session is CONNECTED.  Wi-Fi or lease loss drops the
   session within one poll, so 0 covers link, lease and broker. */
int cyw43_utk_mqtt_connected(void);
/* Sessions lost since boot (wraps); a change shows a drop between reads. */
uint32_t cyw43_utk_mqtt_sessions_lost(void);
/* Ask the radio task to drop a CONNECTED session and reconnect (a stalled
   session that still looks connected).  Never blocks; no-op without MQTT. */
void cyw43_utk_mqtt_restart(void);
void cyw43_utk_task(int32_t stacd, void *exinf);
void cyw43_utk_link_state_changed(int up);

/* Queue one packed record for publication on topic CYW43_MQTT_TOPIC_*.
   Callable from any task; never blocks and never touches lwIP.  Returns 0
   (E_OK) when queued, or a negative kernel error: E_TMOUT when the queue is
   full, E_NOEXS when MQTT is not built in, E_PAR for a bad topic or length. */
int32_t cyw43_utk_mqtt_send(uint32_t topic, const uint8_t *payload,
                            uint32_t length);

#endif
