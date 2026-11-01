/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Wireless console (development profile WIFI_NETLOG=1, never the demo build).
 *
 * Every console byte (tm_snd_dat) is also copied into a ring in tm_com.c;
 * the radio-owner task drains it as UDP datagrams to the laptop, so the
 * viewer gets the full serial console with no cable.  Copying never waits:
 * when the ring is full the bytes are dropped and counted.  Only the drain
 * touches lwIP, and only from the radio task (lwIP is NO_SYS).
 *
 * The ring (kernel side, tm_com.c) and the sender (lwIP side,
 * lwip_utk_netlog.c) are separate files because the kernel and lwIP headers
 * clash (size_t), as in cyw43_utk.c / cyw43_utk_kernel.c.  This header uses
 * only <stdint.h> so both sides can include it.
 *
 * Datagram: 8-byte header, then console text (whole lines when possible).
 *   bytes 0-3  sequence number, little-endian, +1 per datagram
 *   bytes 4-7  console bytes dropped so far (ring full), little-endian
 * A sequence gap at the viewer = datagrams lost on the air (UDP, no retry).
 */
#ifndef LWIP_UTK_NETLOG_H
#define LWIP_UTK_NETLOG_H

#include <stdint.h>

struct netif;

#define LWIP_UTK_NETLOG_HDR     8U
#define LWIP_UTK_NETLOG_TEXT    1024U   /* text bytes per datagram, at most */

/* ---- ring, tm_com.c (any context, any core, never waits) ---- */
void     lwip_utk_netlog_put(const uint8_t *buf, int32_t size);
/* Copy up to max unsent bytes to out without taking them; returns the count. */
uint32_t lwip_utk_netlog_peek(uint8_t *out, uint32_t max);
/* Mark n bytes (from the last peek) as sent. */
void     lwip_utk_netlog_take(uint32_t n);
uint32_t lwip_utk_netlog_dropped(void);   /* bytes dropped, ring full */

/* ---- sender, lwip_utk_netlog.c (radio-owner task only) ---- */
/* Send what the ring holds to dst:port (dst as four octets; 0.0.0.0 =
   nowhere yet, keep it). */
void     lwip_utk_netlog_poll(struct netif *netif, const uint8_t dst[4],
                              uint16_t port);
uint32_t lwip_utk_netlog_sent(void);      /* datagrams sent */

#endif /* LWIP_UTK_NETLOG_H */
