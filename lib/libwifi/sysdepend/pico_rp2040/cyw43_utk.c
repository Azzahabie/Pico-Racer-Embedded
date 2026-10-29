/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Core-0-owned CYW43439 polling service for micro T-Kernel. */

#include "cyw43.h"
#include "cyw43_country.h"
#include "cyw43_utk.h"

#if TM_WIFI_NETIF
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#endif
#if TM_WIFI_DHCP
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#endif
#if TM_WIFI_UDP
#include "lwip_utk_udp.h"
#endif
#if TM_WIFI_TCP
#include "lwip_utk_tcp.h"
#endif
#if TM_WIFI_TCPBULK
#include "lwip_utk_tcpbulk.h"
#endif
#if TM_WIFI_MQTT
#include "lwip_utk_mqtt.h"
#endif
#if TM_WIFI_STATIC || TM_WIFI_DHCP
#include "lwip_utk_ipv4.h"
#endif
#if TM_WIFI_STATIC
#include "network_config.h"
#endif

#define CYW43_POLL_MS        2
/* Bounded so a reader can never wedge if the writer is starved; an exhausted
 * budget returns the last copy and is visible through status_retries. */
#define CYW43_STATUS_READ_ATTEMPTS 16
/* Connection state machine timing.  Overridable so they can be tuned from the
 * build without touching the logic.  The lease timeout is not the 10 s first
 * proposed: on Pico2_IoT a lease takes up to ~12 s after link up, nearly all
 * of it lwIP's ACD address check. */
#ifndef CYW43_JOIN_TIMEOUT_US
#define CYW43_JOIN_TIMEOUT_US 15000000ULL
#endif
#ifndef CYW43_DHCP_TIMEOUT_US
#define CYW43_DHCP_TIMEOUT_US 20000000ULL
#endif
#ifndef CYW43_RETRY_BACKOFF_US
#define CYW43_RETRY_BACKOFF_US 3000000ULL
#endif
#define CYW43_DNS_TIMEOUT_US  30000000ULL

#if TM_WIFI_JOIN
#include "wifi_credentials.h"

static const uint8_t wifi_ssid[] = UTK_WIFI_SSID;
static const uint8_t wifi_password[] = UTK_WIFI_PASSWORD;

_Static_assert(sizeof(wifi_ssid) > 1, "UTK_WIFI_SSID must not be empty");
_Static_assert(sizeof(wifi_ssid) - 1 <= 32, "UTK_WIFI_SSID exceeds 32 bytes");
_Static_assert(UTK_WIFI_AUTH == CYW43_AUTH_OPEN
            || (sizeof(wifi_password) - 1 >= 8
             && sizeof(wifi_password) - 1 <= 63),
               "WPA/WPA2/WPA3 password must contain 8 to 63 bytes");
#endif

extern volatile uint32_t cyw43_utk_poll_requested;
#include "smp_compat.h"
extern int32_t tk_dly_tsk(uint32_t delay);
extern void mp_memory_barrir(void);
extern uint64_t time_us_64(void);

/* Seqlock guarding radio_status.  The radio-owner task publishes field by
 * field on processor 1 while the qualification harness reads the whole
 * structure from the other processor, so an unguarded copy can mix fields
 * from two different updates.  Odd means a publication is in progress. */
static volatile uint32_t status_version;
static volatile uint32_t status_read_retries;

static T_CYW43_UTK_STATUS radio_status = {
    .init_result = -5,
    .strongest_rssi = -32768,
    .join_enabled = TM_WIFI_JOIN,
    .join_request_result = -5,
    .join_result = -5,
    .link_status = CYW43_LINK_DOWN,
    .link_rssi = -32768,
    .dhcp_enabled = TM_WIFI_DHCP,
    .dhcp_result = -5,
    .dns_enabled = TM_WIFI_DNS,
    .dns_request_result = -5,
    .dns_result = -3,
    .udp_enabled = TM_WIFI_UDP,
    .udp_init_result = -6,
    .udp_send_result = -6,
    .udp_result = -3,
    .tcp_enabled = TM_WIFI_TCP,
    .tcp_init_result = -6,
    .tcp_connect_result = -6,
    .tcp_send_result = -6,
    .tcp_close_result = -6,
    .tcp_result = -3,
    .bulk_enabled = TM_WIFI_TCPBULK,
    .bulk_init_result = -6,
    .bulk_connect_result = -6,
    .bulk_send_result = -6,
    .bulk_close_result = -6,
    .bulk_result = -3,
    .ipv4_config_result = -6,
    .ping_send_result = -6,
    .ping_result = -3,
};

static void status_publish_begin(void)
{
    status_version++;
    mp_memory_barrir();
}

static void status_publish_end(void)
{
    mp_memory_barrir();
    status_version++;
}

#if TM_WIFI_NETIF
static void update_netif_status(void)
{
    struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];

    radio_status.netif_initialized = (netif->state == &cyw43_state);
    radio_status.netif_up = netif_is_up(netif);
    radio_status.netif_link_up = netif_is_link_up(netif);
    radio_status.netif_default = (netif_default == netif);
    radio_status.netif_callbacks_ready =
            netif->input != NULL && netif->output != NULL
         && netif->linkoutput != NULL;
    radio_status.netif_no_sys = NO_SYS;
    radio_status.netif_mtu = netif->mtu;
    for(unsigned int i = 0; i < sizeof(radio_status.netif_mac); i++) {
        radio_status.netif_mac[i] = netif->hwaddr[i];
    }
    radio_status.link_up = radio_status.netif_link_up;
}
#endif

#if TM_WIFI_UDP
/* The mirror in cyw43_utk.h duplicates the ring geometry, because that header
   is included by code that cannot see the lwIP headers.  Catch any drift here
   rather than silently truncating echoes. */
typedef char cyw43_ring_size_matches[
	(LWIP_UTK_UDP_RING == CYW43_UDP_RING
	 && LWIP_UTK_UDP_PAYLOAD_SIZE == CYW43_UDP_PAYLOAD) ? 1 : -1];

static void update_udp_status(void)
{
    int i;
    T_LWIP_UTK_UDP_STATUS status;

    lwip_utk_udp_get_status(&status);
    radio_status.udp_init_result = status.init_result;
    radio_status.udp_initialized = status.initialized;
    radio_status.udp_started = status.started;
    radio_status.udp_complete = status.complete;
    radio_status.udp_closed = status.closed;
    radio_status.udp_send_result = status.send_result;
    radio_status.udp_result = status.result;
    radio_status.udp_expected_packets = status.expected_packets;
    radio_status.udp_payload_size = status.payload_size;
    radio_status.udp_packets_sent = status.packets_sent;
    radio_status.udp_packets_received = status.packets_received;
    radio_status.udp_packets_matched = status.packets_matched;
    radio_status.udp_retries = status.retries;
    radio_status.udp_send_errors = status.send_errors;
    radio_status.udp_corrupt = status.corrupt;
    radio_status.udp_unexpected = status.unexpected;
    radio_status.udp_elapsed_ms = status.elapsed_ms;
    radio_status.udp_port = status.port;
    /* Only touch the ring when it has actually advanced.  This runs on the
       2 ms poll cadence, and the ring is ~900 bytes -- copying it every time
       would be pure waste. */
    if(status.write_seq != radio_status.udp_write_seq) {
        int slot, b;
        for(slot = 0; slot < (int)LWIP_UTK_UDP_RING; slot++) {
            radio_status.udp_ring_seq[slot] = status.ring_seq[slot];
            radio_status.udp_ring_len[slot] = status.ring_len[slot];
            for(b = 0; b < (int)LWIP_UTK_UDP_PAYLOAD_SIZE; b++) {
                radio_status.udp_ring_payload[slot][b] =
                    status.ring_payload[slot][b];
            }
        }
        radio_status.udp_write_seq = status.write_seq;
    }
    radio_status.udp_session_count = status.session_count;
    (void)i;
    for(unsigned int i = 0; i < sizeof(status.target); i++) {
        radio_status.udp_target[i] = status.target[i];
    }
}
#endif

#if TM_WIFI_TCP
static void update_tcp_status(void)
{
    T_LWIP_UTK_TCP_STATUS status;

    lwip_utk_tcp_get_status(&status);
    radio_status.tcp_init_result = status.init_result;
    radio_status.tcp_connect_result = status.connect_result;
    radio_status.tcp_send_result = status.send_result;
    radio_status.tcp_close_result = status.close_result;
    radio_status.tcp_result = status.result;
    radio_status.tcp_fatal_error = status.fatal_error;
    radio_status.tcp_initialized = status.initialized;
    radio_status.tcp_started = status.started;
    radio_status.tcp_connected = status.connected;
    radio_status.tcp_complete = status.complete;
    radio_status.tcp_fin_sent = status.fin_sent;
    radio_status.tcp_peer_fin = status.peer_fin;
    radio_status.tcp_closed = status.closed;
    radio_status.tcp_aborted = status.aborted;
    radio_status.tcp_expected_records = status.expected_records;
    radio_status.tcp_record_size = status.record_size;
    radio_status.tcp_records_sent = status.records_sent;
    radio_status.tcp_records_matched = status.records_matched;
    radio_status.tcp_bytes_sent = status.bytes_sent;
    radio_status.tcp_bytes_acked = status.bytes_acked;
    radio_status.tcp_bytes_received = status.bytes_received;
    radio_status.tcp_segments_received = status.segments_received;
    radio_status.tcp_short_segments = status.short_segments;
    radio_status.tcp_corrupt = status.corrupt;
    radio_status.tcp_send_errors = status.send_errors;
    radio_status.tcp_poll_calls = status.poll_calls;
    radio_status.tcp_guard_blocked = status.guard_blocked;
    radio_status.tcp_start_delay_ms = status.start_delay_ms;
    radio_status.tcp_connect_ms = status.connect_ms;
    radio_status.tcp_close_ms = status.close_ms;
    radio_status.tcp_elapsed_ms = status.elapsed_ms;
    radio_status.tcp_port = status.port;
    for(unsigned int i = 0; i < sizeof(status.target); i++) {
        radio_status.tcp_target[i] = status.target[i];
    }
}
#endif

#if TM_WIFI_MQTT
/* The mirror in cyw43_utk.h duplicates the topic/payload capacity, the topic
   indices and the session states for the same reason as the UDP ring
   geometry: that header is seen by code which cannot include the lwIP
   headers.  Catch drift here rather than silently misrouting a record. */
typedef char cyw43_mqtt_size_matches[
	(LWIP_UTK_MQTT_TOPIC_MAX == CYW43_MQTT_TOPIC_MAX
	 && LWIP_UTK_MQTT_PAYLOAD_MAX == CYW43_MQTT_PAYLOAD_MAX
	 && CYW43_MQTT_RECORD_MAX <= CYW43_MQTT_PAYLOAD_MAX) ? 1 : -1];
typedef char cyw43_mqtt_ids_match[
	(LWIP_UTK_MQTT_TOPIC_TELEMETRY == CYW43_MQTT_TOPIC_TELEMETRY
	 && LWIP_UTK_MQTT_TOPIC_HEARTBEAT == CYW43_MQTT_TOPIC_HEARTBEAT
	 && LWIP_UTK_MQTT_TOPIC_STATE == CYW43_MQTT_TOPIC_STATE
	 && LWIP_UTK_MQTT_WAIT_ADDR == CYW43_MQTT_WAIT_ADDR
	 && LWIP_UTK_MQTT_CONNECTING == CYW43_MQTT_CONNECTING
	 && LWIP_UTK_MQTT_CONNECTED == CYW43_MQTT_CONNECTED
	 && LWIP_UTK_MQTT_BACKOFF == CYW43_MQTT_BACKOFF
	 && LWIP_UTK_MQTT_FAILED == CYW43_MQTT_FAILED
	 && LWIP_UTK_MQTT_IN_CMD == CYW43_MQTT_IN_CMD
	 && LWIP_UTK_MQTT_IN_MAP == CYW43_MQTT_IN_MAP
	 && LWIP_UTK_MQTT_IN_MAX == CYW43_MQTT_IN_MAX) ? 1 : -1];

/* The queue lives in cyw43_utk_kernel.c, which can see the kernel API. */
extern int32_t cyw43_utk_mqtt_take(uint32_t *topic, uint8_t *payload);
extern uint32_t cyw43_utk_mqtt_dropped(void);
extern int32_t cyw43_utk_mqtt_post_cmd(uint32_t topic, const uint8_t *payload,
                                       uint32_t length);

/* Hand at most one queued record to the client per poll.  While the session
   is down the records stay queued; producers see a full queue and drop. */
static void drain_mqtt_queue(void)
{
    uint8_t payload[CYW43_MQTT_RECORD_MAX];
    uint32_t topic;
    int32_t length;

    if(!lwip_utk_mqtt_ready()) return;
    length = cyw43_utk_mqtt_take(&topic, payload);
    if(length > 0) {
        (void)lwip_utk_mqtt_offer(topic, payload, (uint32_t)length);
    }
}

static void update_mqtt_status(void)
{
    T_LWIP_UTK_MQTT_STATUS status;

    lwip_utk_mqtt_get_status(&status);
    radio_status.mqtt_enabled = TM_WIFI_MQTT;
    radio_status.mqtt_init_result = status.init_result;
    radio_status.mqtt_connect_request = status.connect_request;
    radio_status.mqtt_connection_result = status.connection_result;
    radio_status.mqtt_publish_result = status.publish_result;
    radio_status.mqtt_publish_ack = status.publish_ack;
    radio_status.mqtt_state = status.state;
    radio_status.mqtt_state_changes = status.state_changes;
    radio_status.mqtt_connect_attempts = status.connect_attempts;
    radio_status.mqtt_connects = status.connects;
    radio_status.mqtt_sessions_lost = status.sessions_lost;
    radio_status.mqtt_publish_attempts = status.publish_attempts;
    radio_status.mqtt_published = status.published;
    radio_status.mqtt_publish_errors = status.publish_errors;
    radio_status.mqtt_poll_calls = status.poll_calls;
    radio_status.mqtt_guard_blocked = status.guard_blocked;
    radio_status.mqtt_connect_ms = status.connect_ms;
    radio_status.mqtt_backoff_ms = status.backoff_ms;
    radio_status.mqtt_queue_dropped = cyw43_utk_mqtt_dropped();
    radio_status.mqtt_subscribed = status.subscribed;
    radio_status.mqtt_in_received = status.inbound_received;
    radio_status.mqtt_in_rejected = status.inbound_rejected;
    radio_status.mqtt_in_dropped = status.inbound_dropped;
    radio_status.mqtt_port = status.port;
    radio_status.mqtt_payload_len = status.payload_len;
    for(unsigned int i = 0; i < sizeof(status.broker); i++) {
        radio_status.mqtt_broker[i] = status.broker[i];
    }
    for(unsigned int i = 0; i < sizeof(status.topic); i++) {
        radio_status.mqtt_topic[i] = status.topic[i];
    }
    for(unsigned int i = 0; i < sizeof(status.payload); i++) {
        radio_status.mqtt_payload[i] = status.payload[i];
    }
}
#endif

#if TM_WIFI_TCPBULK
static void update_bulk_status(void)
{
    T_LWIP_UTK_TCPBULK_STATUS status;

    lwip_utk_tcpbulk_get_status(&status);
    radio_status.bulk_init_result = status.init_result;
    radio_status.bulk_connect_result = status.connect_result;
    radio_status.bulk_send_result = status.send_result;
    radio_status.bulk_close_result = status.close_result;
    radio_status.bulk_result = status.result;
    radio_status.bulk_fatal_error = status.fatal_error;
    radio_status.bulk_initialized = status.initialized;
    radio_status.bulk_started = status.started;
    radio_status.bulk_connected = status.connected;
    radio_status.bulk_complete = status.complete;
    radio_status.bulk_fin_sent = status.fin_sent;
    radio_status.bulk_peer_fin = status.peer_fin;
    radio_status.bulk_closed = status.closed;
    radio_status.bulk_aborted = status.aborted;
    radio_status.bulk_total_bytes = status.total_bytes;
    radio_status.bulk_bytes_sent = status.bytes_sent;
    radio_status.bulk_bytes_acked = status.bytes_acked;
    radio_status.bulk_bytes_received = status.bytes_received;
    radio_status.bulk_mismatches = status.mismatches;
    radio_status.bulk_first_mismatch = status.first_mismatch;
    radio_status.bulk_segments_received = status.segments_received;
    radio_status.bulk_max_segment = status.max_segment;
    radio_status.bulk_min_segment = status.min_segment;
    radio_status.bulk_split_boundaries = status.split_boundaries;
    radio_status.bulk_write_retries = status.write_retries;
    radio_status.bulk_connect_ms = status.connect_ms;
    radio_status.bulk_close_ms = status.close_ms;
    radio_status.bulk_elapsed_ms = status.elapsed_ms;
    radio_status.bulk_throughput_bps = status.throughput_bps;
    radio_status.bulk_port = status.port;
    for(unsigned int i = 0; i < sizeof(status.target); i++) {
        radio_status.bulk_target[i] = status.target[i];
    }
}
#endif

#if TM_WIFI_DNS
static void copy_ip4(uint8_t dst[4], const ip4_addr_t *src);
static uint64_t dns_started_us;

static void dns_found(const char *name, const ip_addr_t *address, void *arg)
{
    (void)name;
    (void)arg;
    /* !dns_started: a late answer to a query from a lease that has since been
     * dropped (lease_reset) must not be reported against the next one. */
    if(!radio_status.dns_started || radio_status.dns_complete) return;

    radio_status.dns_elapsed_ms =
            (uint32_t)((time_us_64() - dns_started_us) / 1000ULL);
    if(address == NULL) {
        radio_status.dns_result = ERR_VAL;
    } else {
        copy_ip4(radio_status.dns_address, ip_2_ip4(address));
        radio_status.dns_result = ERR_OK;
    }
    mp_memory_barrir();
    radio_status.dns_complete = 1;
}

static void update_dns_status(void)
{
    uint64_t now = time_us_64();

    if(!radio_status.dns_started && radio_status.dhcp_complete
       && radio_status.dhcp_result == ERR_OK) {
        ip_addr_t address;
        err_t result;

        radio_status.dns_started = 1;
        dns_started_us = now;
        result = dns_gethostbyname(CYW43_UTK_DNS_TEST_HOSTNAME, &address,
                                   dns_found, NULL);
        radio_status.dns_request_result = result;
        if(result == ERR_OK) {
            copy_ip4(radio_status.dns_address, ip_2_ip4(&address));
            radio_status.dns_result = ERR_OK;
            mp_memory_barrir();
            radio_status.dns_complete = 1;
        } else if(result != ERR_INPROGRESS) {
            radio_status.dns_result = result;
            mp_memory_barrir();
            radio_status.dns_complete = 1;
        }
    }

    if(radio_status.dns_started && !radio_status.dns_complete) {
        radio_status.dns_elapsed_ms =
                (uint32_t)((now - dns_started_us) / 1000ULL);
        if(now - dns_started_us >= CYW43_DNS_TIMEOUT_US) {
            radio_status.dns_result = ERR_TIMEOUT;
            mp_memory_barrir();
            radio_status.dns_complete = 1;
        }
    }
}
#endif


#if TM_WIFI_STATIC || TM_WIFI_DHCP
static void update_ipv4_status(void)
{
    T_LWIP_UTK_IPV4_STATUS status;

    lwip_utk_ipv4_get_status(&status);
    radio_status.ipv4_config_result = status.config_result;
    radio_status.ipv4_configured = status.configured;
    radio_status.arp_resolved = status.arp_resolved;
    radio_status.ping_started = status.ping_started;
    radio_status.ping_complete = status.ping_complete;
    radio_status.ping_send_result = status.ping_send_result;
    radio_status.ping_result = status.ping_result;
    radio_status.ping_attempts = status.ping_attempts;
    radio_status.ping_elapsed_ms = status.ping_elapsed_ms;
    for(unsigned int i = 0; i < sizeof(status.address); i++) {
        radio_status.ipv4_address[i] = status.address[i];
        radio_status.ipv4_netmask[i] = status.netmask[i];
        radio_status.ipv4_gateway[i] = status.gateway[i];
        radio_status.ping_target[i] = status.target[i];
    }
    for(unsigned int i = 0; i < sizeof(status.arp_mac); i++) {
        radio_status.arp_mac[i] = status.arp_mac[i];
    }
}
#endif

#if TM_WIFI_DHCP
static uint64_t dhcp_started_us;
static uint32_t dhcp_ipv4_adopted;

static void copy_ip4(uint8_t dst[4], const ip4_addr_t *src)
{
    dst[0] = ip4_addr1(src);
    dst[1] = ip4_addr2(src);
    dst[2] = ip4_addr3(src);
    dst[3] = ip4_addr4(src);
}

/* True while lwIP holds a usable lease right now: link up, bound, with
 * address, netmask, gateway and a DNS server all set.  Read from the netif
 * rather than from dhcp_complete, so it follows the lease across a rejoin.
 * The link check matters: lwIP keeps the client BOUND while the link is down
 * (F1.1), so without it a dropped link would re-adopt the old lease during the
 * back-off.  On link up lwIP leaves BOUND at once (REBOOTING), so the old lease
 * cannot pass here until the AP confirms it again. */
static int lease_valid(struct netif *netif)
{
    const ip_addr_t *dns = dns_getserver(0);

    return netif_is_link_up(netif)
        && dhcp_supplied_address(netif)
        && !ip4_addr_isany_val(*netif_ip4_addr(netif))
        && !ip4_addr_isany_val(*netif_ip4_netmask(netif))
        && !ip4_addr_isany_val(*netif_ip4_gw(netif))
        && dns != NULL && !ip4_addr_isany_val(*ip_2_ip4(dns));
}

static void update_dhcp_status(void)
{
    struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];
    struct dhcp *dhcp = netif_dhcp_data(netif);
    const ip_addr_t *dns = dns_getserver(0);
    uint64_t now = time_us_64();

    radio_status.dhcp_started = (dhcp != NULL);
    if(dhcp != NULL) {
        radio_status.dhcp_tries = dhcp->tries;
        radio_status.dhcp_state = dhcp->state;
    }
    if(radio_status.netif_link_up && dhcp_started_us == 0) {
        dhcp_started_us = now;
    }
    if(dhcp_started_us != 0 && !radio_status.dhcp_complete) {
        radio_status.dhcp_elapsed_ms =
                (uint32_t)((now - dhcp_started_us) / 1000ULL);
    }

    /* Link check for the same reason as in lease_valid(): while the link is
     * down lwIP still reports the old lease as supplied, and copying it would
     * show a stale address on the console during the back-off. */
    if(!radio_status.dhcp_complete && radio_status.netif_link_up
       && dhcp_supplied_address(netif)) {
        copy_ip4(radio_status.dhcp_address, netif_ip4_addr(netif));
        copy_ip4(radio_status.dhcp_netmask, netif_ip4_netmask(netif));
        copy_ip4(radio_status.dhcp_gateway, netif_ip4_gw(netif));
        if(dns != NULL) {
            copy_ip4(radio_status.dhcp_dns, ip_2_ip4(dns));
        }
        if(lease_valid(netif)) {
            if(!dhcp_ipv4_adopted) {
                radio_status.ipv4_config_result =
                        lwip_utk_ipv4_adopt_netif(netif);
                dhcp_ipv4_adopted = 1;
            }
            radio_status.dhcp_result = ERR_OK;
            mp_memory_barrir();
            radio_status.dhcp_complete = 1;
        }
    }
    /* No timeout here any more: the connection state machine owns it, and a
     * missed lease leads to a rejoin rather than a permanent ERR_TIMEOUT. */
}

/* Forget the current lease so the next one is timed, adopted, reported and
 * resolved afresh.  dhcp_complete therefore means "holds a lease now", not
 * "DHCP finished once".  Called on every join attempt and whenever the car
 * leaves ONLINE.  lwIP's own DHCP client keeps running underneath and is not
 * touched: it restarts discovery by itself on every link up (F1.1). */
static void lease_reset(void)
{
    radio_status.dhcp_complete = 0;
    radio_status.dhcp_result = -5;
    radio_status.dhcp_elapsed_ms = 0;
    for(unsigned int i = 0; i < 4; i++) {
        radio_status.dhcp_address[i] = 0;
        radio_status.dhcp_netmask[i] = 0;
        radio_status.dhcp_gateway[i] = 0;
        radio_status.dhcp_dns[i] = 0;
    }
    dhcp_started_us = 0;
    dhcp_ipv4_adopted = 0;
#if TM_WIFI_DNS
    radio_status.dns_started = 0;
    radio_status.dns_complete = 0;
    radio_status.dns_request_result = -5;
    radio_status.dns_result = -3;
    radio_status.dns_elapsed_ms = 0;
    for(unsigned int i = 0; i < 4; i++) radio_status.dns_address[i] = 0;
    dns_started_us = 0;
#endif
}
#endif

#if TM_WIFI_JOIN
/* Connection state machine (Statechart 6.1, WAITING FOR WI-FI).  Replaces the
 * old one-shot join: any failure -- join error, join timeout, no lease, or the
 * link dropping -- goes to BACKOFF, which leaves the AP and rejoins.  A full
 * leave + rejoin also resets lwIP's DHCP back-off (tries = 0 on link up), which
 * would otherwise grow to 60 s between DISCOVERs.  Only checks timestamps and
 * flags; it never waits inside the poll loop.  States and reasons are the
 * CYW43_NET_* values in cyw43_utk.h. */
static uint32_t net_state = CYW43_NET_SCANNING;
static uint64_t net_state_us;           /* when net_state was entered */

#if !TM_WIFI_DHCP
static void lease_reset(void) {}        /* no lease to forget */
#endif

/* Every transition goes through here, so the counters and net_log can never
 * disagree with the state.  Runs inside the status publication. */
static void net_enter(uint32_t next, uint32_t why, int32_t code, uint64_t now)
{
    T_CYW43_UTK_NET_EVENT *ev =
            &radio_status.net_log[radio_status.net_log_seq % CYW43_NET_LOG];

    net_state = next;
    net_state_us = now;
    if(next == CYW43_NET_WAIT_LEASE) radio_status.lease_attempts++;
    if(why == CYW43_NET_WHY_LINK_LOST) radio_status.link_losses++;
    radio_status.net_state = next;

    ev->t_ms = (uint32_t)(now / 1000ULL);
    ev->state = next;
    ev->why = why;
    ev->code = code;
    ev->join_attempts = radio_status.join_attempts;
    ev->lease_attempts = radio_status.lease_attempts;
    ev->link_losses = radio_status.link_losses;
    radio_status.net_log_seq++;
}

/* Leave on entry rather than after the wait, so a slow association cannot
 * complete in the middle of the back-off.  Harmless when not associated. */
static void net_backoff(uint32_t why, int32_t code, uint64_t now)
{
    (void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    net_enter(CYW43_NET_BACKOFF, why, code, now);
}

static void join_start(uint64_t now)
{
    int32_t result;

    lease_reset();
    radio_status.join_attempts++;
    radio_status.join_started = 1;
    radio_status.join_complete = 0;
    radio_status.link_up = 0;
    radio_status.link_status = CYW43_LINK_DOWN;
    result = cyw43_wifi_join(&cyw43_state,
                             sizeof(wifi_ssid) - 1, wifi_ssid,
                             sizeof(wifi_password) - 1, wifi_password,
                             UTK_WIFI_AUTH, NULL,
                             CYW43_CHANNEL_NONE);
    radio_status.join_request_result = result;
    if(result != 0) {
        radio_status.join_result = result;
        mp_memory_barrir();
        radio_status.join_complete = 1;
        net_backoff(CYW43_NET_WHY_JOIN_REQUEST, result, now);
    } else {
        net_enter(CYW43_NET_JOINING, CYW43_NET_WHY_NONE, 0, now);
    }
}

/* why is only used when join_result != 0. */
static void join_finish(int32_t join_result, uint32_t why, uint64_t now)
{
    radio_status.join_result = join_result;
    mp_memory_barrir();
    radio_status.join_complete = 1;
    if(join_result != 0) net_backoff(why, join_result, now);
}

static void update_net_state(uint64_t now)
{
#if TM_WIFI_DHCP
    struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];
#endif

    switch(net_state) {
    case CYW43_NET_SCANNING:
        if(radio_status.scan_complete) join_start(now);
        break;

    case CYW43_NET_JOINING: {
        int32_t link = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

        radio_status.join_elapsed_ms = (uint32_t)
                ((now - net_state_us) / 1000ULL);
        radio_status.link_status = link;
        if(radio_status.link_up) {
            (void)cyw43_wifi_get_bssid(&cyw43_state, radio_status.bssid);
            (void)cyw43_wifi_get_rssi(&cyw43_state,
                                      (int32_t *)&radio_status.link_rssi);
            join_finish(0, CYW43_NET_WHY_NONE, now);
#if TM_WIFI_DHCP
            net_enter(CYW43_NET_WAIT_LEASE, CYW43_NET_WHY_NONE, 0, now);
#else
            net_enter(CYW43_NET_ONLINE, CYW43_NET_WHY_NONE, 0, now);
#endif
        } else if(link < CYW43_LINK_DOWN) {
            join_finish(link, CYW43_NET_WHY_JOIN_FAILED, now);
        } else if(now - net_state_us >= CYW43_JOIN_TIMEOUT_US) {
            join_finish(-CYW43_ETIMEDOUT, CYW43_NET_WHY_JOIN_TIMEOUT, now);
        }
        break;
    }

#if TM_WIFI_DHCP
    case CYW43_NET_WAIT_LEASE:
        if(!radio_status.link_up) {
            net_backoff(CYW43_NET_WHY_LINK_LOST, 0, now);
        } else if(lease_valid(netif)) {
            net_enter(CYW43_NET_ONLINE, CYW43_NET_WHY_NONE, 0, now);
        } else if(now - net_state_us >= CYW43_DHCP_TIMEOUT_US) {
            net_backoff(CYW43_NET_WHY_LEASE_TIMEOUT, 0, now);
        }
        break;
#endif

    case CYW43_NET_ONLINE:
        /* Clear the lease on the way out, not at the next join_start(), so
         * nothing gated on dhcp_complete runs during the back-off. */
        if(!radio_status.link_up) {
            lease_reset();
            net_backoff(CYW43_NET_WHY_LINK_LOST, 0, now);
#if TM_WIFI_DHCP
        } else if(!lease_valid(netif)) {
            lease_reset();
            net_enter(CYW43_NET_WAIT_LEASE, CYW43_NET_WHY_LEASE_LOST, 0, now);
#endif
        }
        break;

    case CYW43_NET_BACKOFF:
        if(now - net_state_us >= CYW43_RETRY_BACKOFF_US) join_start(now);
        break;

    default:
        break;
    }
}
#endif

void cyw43_utk_link_state_changed(int up)
{
    radio_status.link_up = (up != 0);
    if(!up) radio_status.link_status = CYW43_LINK_DOWN;
    mp_memory_barrir();
}

static int scan_result(void *env, const cyw43_ev_scan_result_t *result)
{
    T_CYW43_UTK_STATUS *status = env;

    if(result == NULL) return 0;
    status->scan_count++;
    if(result->rssi > status->strongest_rssi) {
        status->strongest_rssi = result->rssi;
    }
    return 0;
}

void cyw43_utk_task(int32_t stacd, void *exinf)
{
    cyw43_wifi_scan_options_t options = {0};
    int32_t result;
    bool was_scanning;

    (void)stacd;
    (void)exinf;
    radio_status.owner_prc = (uint32_t)CYW43_THIS_PRC();

#if TM_WIFI_NETIF
    lwip_init();
    radio_status.lwip_initialized = 1;
#endif
#if TM_WIFI_MQTT
    /* Inbound commands go straight to comms_task's mailbox (design 9.3). */
    lwip_utk_mqtt_set_inbound(cyw43_utk_mqtt_post_cmd);
#endif
    cyw43_init(&cyw43_state);
    cyw43_thread_enter();
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true,
                      CYW43_COUNTRY_WORLDWIDE);
    if((cyw43_state.itf_state & (1u << CYW43_ITF_STA)) == 0) {
        /* cyw43_wifi_set_up() has a void API and otherwise hides its bus-init
         * error.  Report the underlying I/O class instead of letting scan()
         * replace it with the secondary "interface not up" (-4) result. */
        result = -CYW43_EIO;
    } else {
        (void)cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA,
                                 radio_status.mac);
        result = cyw43_wifi_scan(&cyw43_state, &options, &radio_status,
                                 scan_result);
    }
    cyw43_thread_exit();

#if TM_WIFI_NETIF
    update_netif_status();
#endif
#if TM_WIFI_STATIC
    radio_status.ipv4_config_result = lwip_utk_ipv4_configure(
            &cyw43_state.netif[CYW43_ITF_STA],
            UTK_IPV4_ADDRESS, UTK_IPV4_NETMASK, UTK_IPV4_GATEWAY,
            UTK_IPV4_PING_TARGET);
    update_ipv4_status();
#endif

    radio_status.init_result = result;
    radio_status.ready = (result == 0);
    radio_status.scan_active = (result == 0);
    mp_memory_barrir();
    radio_status.init_complete = 1;

    /* A failed bus has nothing useful to poll.  More importantly, repeatedly
     * waking a failed high-rate service must not interfere with USB or the
     * rest of the kernel qualification. */
    if(result != 0) {
        for(;;) (void)tk_dly_tsk(1000);
    }

    for(;;) {
        was_scanning = radio_status.scan_active != 0;
        cyw43_thread_enter();
        radio_status.poll_stage = 1;     /* diagnostic: driver poll */
        if(cyw43_poll != NULL) cyw43_poll();
        status_publish_begin();
#if TM_WIFI_NETIF
        radio_status.poll_stage = 2;     /* lwIP timers (DHCP, ACD, DNS) */
        sys_check_timeouts();
        update_netif_status();
#endif
#if TM_WIFI_DHCP
        radio_status.poll_stage = 3;
        update_dhcp_status();
#endif
#if TM_WIFI_DNS
        radio_status.poll_stage = 4;
        update_dns_status();
#endif
#if TM_WIFI_UDP
        if(radio_status.dns_complete && radio_status.dns_result == ERR_OK) {
            lwip_utk_udp_poll(&cyw43_state.netif[CYW43_ITF_STA]);
        }
        update_udp_status();
#endif
#if TM_WIFI_TCP
        /* Sequenced after UDP so one capture never overlaps two traffic
         * patterns, but deliberately not gated on the UDP *result*: TCP shares
         * no state with it, and a missing UDP peer must not hide the TCP
         * verdict behind an unrelated failure. */
        if(radio_status.udp_complete) {
            lwip_utk_tcp_poll(&cyw43_state.netif[CYW43_ITF_STA]);
        }
        update_tcp_status();
#endif
#if TM_WIFI_TCPBULK
        /* Sequenced after the Phase-7 session so the peer script is serving
         * one connection at a time, and again not gated on its result. */
        if(radio_status.tcp_complete) {
            lwip_utk_tcpbulk_poll(&cyw43_state.netif[CYW43_ITF_STA]);
        }
        update_bulk_status();
#endif
#if TM_WIFI_MQTT
        /* Polled every pass, not gated on the lease: the client must see the
         * address disappear to drop its session, and see it return to
         * reconnect.  It guards on the netif state itself.  Not sequenced
         * behind the UDP/TCP echo phases either -- MQTT is a client for the
         * whole run, not another link check. */
        lwip_utk_mqtt_poll(&cyw43_state.netif[CYW43_ITF_STA]);
        drain_mqtt_queue();
        update_mqtt_status();
#endif
#if TM_WIFI_STATIC || TM_WIFI_DHCP
        radio_status.poll_stage = 5;
        lwip_utk_ipv4_poll(&cyw43_state.netif[CYW43_ITF_STA]);
        update_ipv4_status();
#endif
        radio_status.poll_stage = 6;     /* join bookkeeping */
        radio_status.poll_count++;
        radio_status.scan_active = cyw43_wifi_scan_active(&cyw43_state);
        if(was_scanning && !radio_status.scan_active) {
            mp_memory_barrir();
            radio_status.scan_complete = 1;
        }
#if TM_WIFI_JOIN
        update_net_state(time_us_64());
#endif
        status_publish_end();
        cyw43_utk_poll_requested = 0;
        radio_status.poll_stage = 7;     /* sleeping until next poll */
        cyw43_thread_exit();
        (void)tk_dly_tsk(CYW43_POLL_MS);
    }
}

void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status)
{
    unsigned int attempt;

    if(status == NULL) return;
    /* Seqlock read.  Barriers alone cannot make a ~230-byte copy atomic, so
     * retry until the version is even (no publication in progress) and
     * unchanged across the copy.  Without this a caller can observe, say, a
     * completion flag from one update beside result fields from an earlier
     * one, and act on a state that never actually existed. */
    for(attempt = 0; attempt < CYW43_STATUS_READ_ATTEMPTS; attempt++) {
        uint32_t before = status_version;

        mp_memory_barrir();
        *status = radio_status;
        mp_memory_barrir();
        if((before & 1U) == 0U && before == status_version) break;
        status_read_retries++;
    }
    status->status_retries = status_read_retries;
}

/* One-word readers for comms_task's Wi-Fi-loss bit, so it need not copy the
 * whole status snapshot every second.  An aligned 32-bit load cannot tear, so
 * no seqlock: the two calls may come from different publications, and the
 * caller treats either one alone as a loss. */
int cyw43_utk_mqtt_connected(void)
{
    return radio_status.mqtt_state == CYW43_MQTT_CONNECTED;
}

uint32_t cyw43_utk_mqtt_state(void)
{
    return radio_status.mqtt_state;
}

uint32_t cyw43_utk_mqtt_sessions_lost(void)
{
    return radio_status.mqtt_sessions_lost;
}

/* Any task: drop a CONNECTED session that has stopped moving and reconnect
 * through the normal back-off.  Only a flag is set here; the radio task acts
 * on it in its next poll, so lwIP stays single-threaded. */
void cyw43_utk_mqtt_restart(void)
{
#if TM_WIFI_MQTT
    lwip_utk_mqtt_request_restart();
#endif
}

/* Inbound messages rejected by the radio (unknown topic, empty, too long)
   or dropped at a full mailbox, since boot. */
uint32_t cyw43_utk_mqtt_in_refused(void)
{
    return radio_status.mqtt_in_rejected + radio_status.mqtt_in_dropped;
}

/* For D: RSSI at the last join (dBm), and the current IPv4 address, first
   octet in the low byte (0 while there is none). */
int32_t cyw43_utk_rssi(void)
{
    return radio_status.link_rssi;
}

uint32_t cyw43_utk_ip4(void)
{
#if TM_WIFI_DHCP
    const uint8_t *a = radio_status.dhcp_address;
#else
    const uint8_t *a = radio_status.ipv4_address;
#endif
    return (uint32_t)a[0] | ((uint32_t)a[1] << 8)
           | ((uint32_t)a[2] << 16) | ((uint32_t)a[3] << 24);
}
