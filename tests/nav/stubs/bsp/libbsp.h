/* Host stand-in for <bsp/libbsp.h> for nav_ctrl.c: the motion sim's stub
   plus the board calls the buttons and buzzer use (signatures from
   include/bsp/sysdepend/cpu/rp2040/libbsp.h), and the kernel constants
   nav_ctrl.c needs that the motion stub does not define. */
#ifndef SIM_NAV_LIBBSP_H
#define SIM_NAV_LIBBSP_H
#include <tk/tkernel.h>
#define GPIO_MODE_OUT	0
#define GPIO_MODE_IN	1
#ifndef TRUE
#define TRUE		1
#define FALSE		0
#endif
#ifndef TA_WMUL
#define TA_WMUL		0x00000008UL
#endif
ER   gpio_set_pin(UINT no, UINT mode);
ER   gpio_set_val(UINT no, UINT val);
UINT gpio_get_val(UINT no);
ER   pwm_set_pin(UINT no);
ER   pwm_set_wrap(UINT no, UW wrap);
ER   pwm_set_cc(UINT no, UW cc);
ER   pwm_set_enabled(UINT no, BOOL enable);
#endif
