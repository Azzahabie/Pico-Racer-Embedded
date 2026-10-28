/* Host stand-in for <tm/tmonitor.h>: tm_printf is provided by the test,
 * which records the lines hump.c prints. */
#ifndef STUB_TM_TMONITOR_H
#define STUB_TM_TMONITOR_H

#include <tk/tkernel.h>

IMPORT INT tm_printf(const UB *format, ...);

#endif
