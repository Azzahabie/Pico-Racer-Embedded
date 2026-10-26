/* Host stand-in for <tm/tmonitor.h>. */
#ifndef SIM_TMONITOR_H
#define SIM_TMONITOR_H
#include <tk/tkernel.h>
INT tm_printf(const UB *format, ...);
INT tm_sprintf(UB *str, const UB *format, ...);
#endif
