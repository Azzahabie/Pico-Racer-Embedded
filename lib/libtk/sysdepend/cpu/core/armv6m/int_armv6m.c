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

#include <sys/machine.h>
#ifdef CPU_CORE_ARMV6M

/*
 *	int_armv6m.c
 *
 *	Interrupt controller (ARMv6-M)
 */

#include <tk/tkernel.h>

#include "int_armv6m.h"

/*----------------------------------------------------------------------*/
/*
 * CPU Interrupt Control for ARMv6-M.
 *
 */
EXPORT UW disint(void)
{
	UW	intsts = get_primask();
	set_primask(1);

	return intsts;
}

/*----------------------------------------------------------------------*/
/*
 * Interrupt controller (NVIC) Control
 *
 */

/*
 * Enable interrupt for NVIC
 */
EXPORT void EnableInt_nvic( UINT intno, INT level )
{
	UINT	imask;
	UW	ipr_adr = NVIC_IPR(intno & ~3U);	/* word holding intno's byte */
	UW	shift = (intno % 4U) * 8U;
	UW	ipr;

	DI(imask);
	/* Set interrupt priority level.  ARMv6-M (Cortex-M0+) allows only
	 * word access to NVIC_IPRn, so update this interrupt's byte with a
	 * word read-modify-write; the other three interrupts in the word keep
	 * their priorities.  Interrupts are masked, and each core has its own
	 * NVIC, so nothing else writes this word meanwhile. */
	ipr = *(_UW*)ipr_adr;
	ipr &= ~(0xFFU << shift);
	ipr |= ((UW)(UB)INTPRI_GROUP(level, 0)) << shift;
	*(_UW*)ipr_adr = ipr;

	/* Enables the specified interrupt. */
	*(_UW*)(NVIC_ISER(intno)) = (0x01U << (intno % 32));

	EI(imask);
}

#endif /* CPU_CORE_ARMV6M */
