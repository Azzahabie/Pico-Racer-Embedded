/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Wireless console sender (lwip_utk_netlog.h): the lwIP side, radio-owner
 * task only.  The ring is in tm_com.c.  Development profile only
 * (WIFI_NETLOG=1). */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "lwip_utk_netlog.h"

#define NETLOG_PER_POLL 2U      /* datagrams per radio pass, at most */

static uint8_t          text[LWIP_UTK_NETLOG_TEXT];
static uint32_t         sent;
static uint32_t         seq;
static struct udp_pcb  *pcb;

uint32_t lwip_utk_netlog_sent(void) { return sent; }

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* One datagram: up to LWIP_UTK_NETLOG_TEXT bytes, cut after the last
   newline when there is one, so lines arrive whole.  0: nothing to send. */
static int send_one(const ip_addr_t *dst, uint16_t port)
{
    uint32_t     n = lwip_utk_netlog_peek(text, LWIP_UTK_NETLOG_TEXT);
    uint32_t     cut = 0U;
    uint32_t     i;
    struct pbuf *p;
    uint8_t     *out;

    if(n == 0U) return 0;
    for(i = 0U; i < n; i++) {
        if(text[i] == (uint8_t)'\n') cut = i + 1U;
    }
    /* A partial line waits until it ends, unless it fills a datagram. */
    if(cut == 0U) {
        if(n < LWIP_UTK_NETLOG_TEXT) return 0;
        cut = n;
    }

    p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(LWIP_UTK_NETLOG_HDR + cut), PBUF_RAM);
    if(p == NULL) return 0;             /* no heap now: retry next pass */
    out = (uint8_t *)p->payload;
    put_le32(out, seq);
    put_le32(out + 4, lwip_utk_netlog_dropped());
    memcpy(out + LWIP_UTK_NETLOG_HDR, text, cut);
    if(udp_sendto(pcb, p, dst, port) == ERR_OK) {
        seq++;
        sent++;
        lwip_utk_netlog_take(cut);      /* only now: a failed send retries */
    }
    (void)pbuf_free(p);
    return 1;
}

void lwip_utk_netlog_poll(struct netif *netif, const uint8_t dst[4],
                          uint16_t port)
{
    ip_addr_t to;
    uint32_t  k;

    if((netif == NULL) || !netif_is_up(netif) || !netif_is_link_up(netif)
       || ip4_addr_isany_val(*netif_ip4_addr(netif))) {
        return;                         /* keep the ring until we can send */
    }
    if((dst[0] | dst[1] | dst[2] | dst[3]) == 0U) return;
    if(pcb == NULL) {
        pcb = udp_new();
        if(pcb == NULL) return;
    }
    IP_ADDR4(&to, dst[0], dst[1], dst[2], dst[3]);
    for(k = 0U; k < NETLOG_PER_POLL; k++) {
        if(!send_one(&to, port)) break;
    }
}
