/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.04
 *
 *    Copyright (C) 2006-2021 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2021/05/17.
 *
 *----------------------------------------------------------------------
 */

/*
 *	config_tm.h
 *	T-Monitor Configuration Definition
 */

#ifndef __TM_CONFIG_H__
#define __TM_CONFIG_H__

/*---------------------------------------------------------------------- */
/* Select a communication port
 *      Select the communication port used by T-Monitor.
 *         1: Valid  0: Invalid  (Only one of them is valid)
 */
#define	TM_COM_SERIAL_DEV	(1)	/* Use serial communication device */
#define	TM_COM_NO_DEV		(0)	/* Do not use communication port */

/*---------------------------------------------------------------------- */
/* tm_printf() call setting
 *         1: Valid  0: Invalid
 */
#define	USE_TM_PRINTF		(1)	/* Use tm_printf() & tm_sprintf() calls */
#define	TM_OUTBUF_SZ		(0)	/* Output Buffer size in stack */

/*
 * Console sink selection (RaspberryPi Pico).
 *
 * TM_CONSOLE_USB_CDC is normally set by the build (CONSOLE=usb_cdc in
 * build_make/pico_rp2040.mk); these are the fallbacks.
 *
 * UART0 (GP0 TX / GP1 RX) is the early-boot, handler and fault console: USB
 * output needs the scheduler and an enumerated host, so it says nothing
 * about the Phase-0 startup window.
 *
 * On the car, GP0/GP1 are the left wheel encoder (approved pin map, Grove 1:
 * "Console over USB CDC, so GP0/GP1 are free").  So the USB-CDC build leaves
 * UART0 off: TX would drive GP0 against the encoder output, and the mirror
 * waits ~87 us per character past the 32-byte FIFO with interrupts masked,
 * which on the motion core delays the encoder interrupt.  For early-boot
 * debugging, build the UART profile (CONSOLE=uart) with the left encoder
 * unplugged.
 */
#ifndef TM_CONSOLE_USB_CDC
#define	TM_CONSOLE_USB_CDC	(0)	/* USB CDC-ACM console */
#endif
#ifndef TM_CONSOLE_UART
#if TM_CONSOLE_USB_CDC
#define	TM_CONSOLE_UART		(0)	/* GP0/GP1 belong to the encoder */
#else
#define	TM_CONSOLE_UART		(1)	/* UART0 console */
#endif
#endif

#endif /* __TM_CONFIG_H__ */
