/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.0 BSP
 *
 *    Copyright (C) 2021-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/11.
 *
 *----------------------------------------------------------------------
 */

/*
 *    tm_com.c
 *    T-Monitor Communication low-level device driver (RaspberryPi Pico / Pico W)
 *
 *    Two sinks, because they are good at different things:
 *
 *      UART0   available from libtm_init() onward, needs no host, no
 *              enumeration and no scheduler.  This is the only channel that
 *              works during early boot, inside handlers, and after a fault,
 *              so it stays compiled in as the early/panic mirror.
 *
 *      USB CDC the console you actually read.  Needs the scheduler (its
 *              service task drains the ring) and a host that has enumerated
 *              the device, so it says nothing about early boot.
 *
 *    Output goes to both when both are enabled.  Neither waits indefinitely
 *    on external I/O: the UART wait is bounded and the USB path only appends
 *    under a short SMP ring lock.  This matters -- the original driver spun
 *    forever on a UART flag, which turns any console problem into an
 *    unexplained hang.
 */

#include <tk/typedef.h>
#include <sys/sysdef.h>

#if USE_TMONITOR
#include "../../libtm.h"

#ifdef PICO_RP2040

/* UART register definition (Use UART0) */
#define UART_BASE              0x40034000

#define UART_DR		(*(_UW*)(UART_BASE+0x000))
#define UART_FR		(*(_UW*)(UART_BASE+0x018))
#define UART_IBRD	(*(_UW*)(UART_BASE+0x024))
#define UART_FBRD	(*(_UW*)(UART_BASE+0x028))
#define UART_LCR_H	(*(_UW*)(UART_BASE+0x02C))
#define UART_CR		(*(_UW*)(UART_BASE+0x030))

#define	FR_TXFF		(1<<5)	// Transmit FIFO full
#define	FR_RXFE		(1<<4)	// Receive FIFO empty

/*
 * Bound on the wait for FIFO space, in loop iterations.  Generous for a
 * 115200 baud character (~87 us) yet still finite, so a UART that is
 * unclocked or held in reset costs a bounded delay and a dropped byte
 * instead of hanging the system.
 */
#define UART_TX_SPIN_LIMIT	200000

#if TM_CONSOLE_UART
LOCAL void uart_snd_dat(const UB* buf, INT size)
{
	UB	*b;
	UW	spin;

	for( b = (UB *)buf; size > 0; size--, b++ ){
		spin = UART_TX_SPIN_LIMIT;
		while( (UART_FR & FR_TXFF) != 0 ){
			if( --spin == 0 ) return;	/* give up, drop the rest */
		}
		UART_DR = *b;
	}
}
#endif /* TM_CONSOLE_UART */

#if TM_CONSOLE_USB_CDC
IMPORT void tm_usb_console_put(const UB *buf, INT size);
#endif
#if defined(TM_WIFI_NETLOG) && TM_WIFI_NETLOG
/* Development profile: every console byte also goes out over Wi-Fi
   (lib/libnet/lwip/lwip_utk_netlog.c).  Never waits. */
#include "lwip_utk_netlog.h"
#include <tk/smp.h>

/* The ring (lwip_utk_netlog.h).  16 KB: the console averaged ~0.8 KB/s on
   the car (S7 attempt 1) with bursts at each scan, so this holds ~20 s and
   boot lines printed before Wi-Fi is up mostly survive to the first send.
   Same lock pattern as the USB console's ring (usb_console.c). */
#define NETLOG_SIZE	16384U
#define NETLOG_MASK	(NETLOG_SIZE - 1U)

LOCAL UB		netlog_ring[NETLOG_SIZE];
LOCAL volatile UW	netlog_head;	/* next byte written */
LOCAL volatile UW	netlog_tail;	/* next byte to send */
LOCAL volatile UW	netlog_dropped;	/* bytes dropped: ring full */
LOCAL T_SPLOCK		netlog_lock;

LOCAL void netlog_enter(UINT *intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinLock(&netlog_lock, intsts);
#else
	DI(*intsts);
#endif
}

LOCAL void netlog_leave(UINT intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinUnlock(&netlog_lock, intsts);
#else
	EI(intsts);
#endif
}

EXPORT void lwip_utk_netlog_put(const uint8_t *buf, int32_t size)
{
	UINT	intsts;
	UW	next;

	if((buf == NULL) || (size <= 0)) return;
	netlog_enter(&intsts);
	while(size-- > 0) {
		next = (netlog_head + 1U) & NETLOG_MASK;
		if(next == netlog_tail) {
			netlog_dropped += (UW)size + 1U;	/* never wait */
			break;
		}
		netlog_ring[netlog_head] = *buf++;
		netlog_head = next;
	}
	netlog_leave(intsts);
}

EXPORT uint32_t lwip_utk_netlog_peek(uint8_t *out, uint32_t max)
{
	UINT	intsts;
	UW	h, t, n, i;

	netlog_enter(&intsts);
	h = netlog_head;
	t = netlog_tail;
	netlog_leave(intsts);
	n = (h - t) & NETLOG_MASK;
	if(n > max) n = max;
	for(i = 0U; i < n; i++) {
		out[i] = netlog_ring[(t + i) & NETLOG_MASK];	/* only the drain moves tail */
	}
	return (uint32_t)n;
}

EXPORT void lwip_utk_netlog_take(uint32_t n)
{
	UINT	intsts;

	netlog_enter(&intsts);
	netlog_tail = (netlog_tail + (UW)n) & NETLOG_MASK;
	netlog_leave(intsts);
}

EXPORT uint32_t lwip_utk_netlog_dropped(void)
{
	return (uint32_t)netlog_dropped;
}
#endif

EXPORT	void	tm_snd_dat( const UB* buf, INT size )
{
#if defined(TM_WIFI_NETLOG) && TM_WIFI_NETLOG
	lwip_utk_netlog_put((const uint8_t *)buf, (int32_t)size);
#endif
#if TM_CONSOLE_USB_CDC
	tm_usb_console_put(buf, size);
#endif
#if TM_CONSOLE_UART
	uart_snd_dat(buf, size);
#endif
}

EXPORT	void	tm_rcv_dat( UB* buf, INT size )
{
#if TM_CONSOLE_UART
	for( ; size > 0; size--, buf++ ){
		while ( (UART_FR & FR_RXFE) != 0 );
		*buf = UART_DR & 0xff;
	}
#else
	/* Console input is not implemented on the USB-only configuration. */
	for( ; size > 0; size--, buf++ ){
		*buf = 0;
	}
#endif
}

EXPORT	void	tm_com_init(void)
{
#if TM_CONSOLE_UART
	UART_IBRD	= 67;			/* Baud rate setting */
	UART_FBRD	= 52;
	UART_LCR_H	= 0x70;			/* Communication data format setting */
	UART_CR 	= (1<<9)|(1<<8)|(1<<0);	/* Communication enabled */
#endif
	/* The USB side needs the scheduler, so it is started later from
	   knl_start_device(); nothing to do here. */
}

#endif /* PICO_RP2040 */
#endif /* USE_TMONITOR */
