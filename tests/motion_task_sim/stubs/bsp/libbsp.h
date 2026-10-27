/* Host stand-in for <bsp/libbsp.h>. */
#ifndef SIM_LIBBSP_H
#define SIM_LIBBSP_H
#include <tk/tkernel.h>
#define GPIO_MODE_OUT	0
#define GPIO_MODE_IN	1
ER gpio_set_pin(UINT no, UINT mode);
ER pwm_set_pin(UINT no);
#endif
