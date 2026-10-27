/* Host stand-in for <tk/tkernel.h>: just the types hump.c uses, at the
 * same widths as on the RP2040 (W and UW are 32 bit there). */
#ifndef STUB_TK_TKERNEL_H
#define STUB_TK_TKERNEL_H

#include <stdint.h>

typedef int32_t		W;
typedef uint32_t	UW;
typedef int		INT;
typedef unsigned int	UINT;
typedef unsigned char	UB;
typedef UINT		BOOL;

#define LOCAL	static
#define EXPORT
#define IMPORT	extern
#define TRUE	1
#define FALSE	0

#endif
