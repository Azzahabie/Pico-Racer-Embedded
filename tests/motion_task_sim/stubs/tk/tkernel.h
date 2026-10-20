/*
 *	Host stand-in for <tk/tkernel.h>: just enough of the µT-Kernel types,
 *	constants and calls for motor_task.c to compile and run on a PC.
 *	The kernel calls are implemented by sim_motion.c.
 */
#ifndef SIM_TKERNEL_H
#define SIM_TKERNEL_H

#include <stdint.h>
#include <stddef.h>

typedef int		INT;
typedef unsigned int	UINT;
typedef int32_t		W;
typedef uint32_t	UW;
typedef uint16_t	UH;
typedef unsigned char	UB;
typedef int		ER;
typedef int		ID;
typedef uint32_t	ATR;
typedef void		(*FP)();
typedef int		PRI;
typedef int		SZ;
typedef uint32_t	RELTIM;
typedef int		TMO;
typedef int		BOOL;

#define LOCAL		static
#define EXPORT
#define IMPORT		extern
#define Inline		static inline
#define CONST		const

#define E_OK		(0)
#define E_PAR		(-17)
#define E_NOMEM		(-33)
#define E_LIMIT		(-34)
#define E_OBJ		(-41)
#define E_TMOUT		(-50)

#define TA_HLNG		0x00000001UL
#define TA_USERBUF	0x00000020UL
#define TA_RNG3		0x00000300UL
#define TA_ASSPRC	0x00080000UL
#define TA_TFIFO	0x00000000UL
#define TA_WSGL		0x00000000UL
#define TA_STA		0x00000002UL
#define TWF_ORW		0x00000001U
#define TWF_BITCLR	0x00000020U
#define TMO_POL		(0)
#define TMO_FEVR	(-1)

#define TK_SUPPORT_SMP		1
#define TP_PRC1			0x00000001
#define TP_PRC2			0x00000002
#define INTPRI_MAX_EXTINT_PRI	1

typedef struct { void *exinf; ATR tskatr; FP task; PRI itskpri; SZ stksz;
		 void *bufptr; UW assprc; } T_CTSK;
typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
typedef struct { void *exinf; ATR mbfatr; SZ bufsz; INT maxmsz; void *bufptr; } T_CMBF;
typedef struct { void *exinf; ATR cycatr; FP cychdr; RELTIM cyctim; RELTIM cycphs; } T_CCYC;
typedef struct { ATR intatr; FP inthdr; } T_DINT;

/* Registers the code under test touches. */
#define TIMER_BASE		0x40054000u
#define SIO_BASE		0xD0000000u
#define GPIO_IN			(SIO_BASE + 0x004u)
#define SIO_CPUID_ADDR		0xd0000000u
#define IO_BANK0_BASE		0x40014000u
#define PADS_BANK0_BASE		0x4001c000u
#define GPIO(n)			(PADS_BANK0_BASE + 0x4u + ((n) * 4u))
#define GPIO_IE			(1u << 6)
#define GPIO_PUE		(1u << 3)
#define GPIO_PDE		(1u << 2)
#define GPIO_SHEMITT		(1u << 1)
#define PWM_BASE		0x40050000u
#define PWM_CHx_CSR		(0x00u)
#define PWM_CHx_DIV		(0x04u)
#define PWM_CHx_CTR		(0x08u)
#define PWM_CHx_CC		(0x0Cu)
#define PWM_CHx_TOP		(0x10u)
#define PWM_CH_CSR_EN		(1u << 0)
#define RESETS_BASE		0x4000C000u
#define RESETS_RESET		(RESETS_BASE + 0x0u)
#define RESETS_RESET_DONE	(RESETS_BASE + 0x8u)
#define RESETS_RESET_PWM	(0x00004000u)

UW   in_w(UW addr);
void out_w(UW addr, UW data);
void set_w(UW addr, UW data);
void clr_w(UW addr, UW data);

#define DI(s)	((s) = 0u)
#define EI(s)	((void)(s))

ID  tk_cre_flg(CONST T_CFLG *p);
ER  tk_set_flg(ID id, UINT ptn);
ER  tk_clr_flg(ID id, UINT ptn);
ER  tk_wai_flg(ID id, UINT ptn, UINT mode, UINT *p_ptn, TMO tmo);
ID  tk_cre_mbf(CONST T_CMBF *p);
ER  tk_snd_mbf(ID id, void *msg, INT size, TMO tmo);
INT tk_rcv_mbf(ID id, void *msg, TMO tmo);
ID  tk_cre_tsk(CONST T_CTSK *p);
ER  tk_sta_tsk(ID id, INT stacd);
ID  tk_cre_cyc(CONST T_CCYC *p);
ER  tk_dly_tsk(TMO tmo);
ER  tk_slp_tsk(TMO tmo);
INT tk_get_prc(void);
ER  tk_def_int(UINT intno, CONST T_DINT *p);
void EnableInt(UINT intno, INT level);
void ClearInt(UINT intno);

#endif
