/*
 * barcode_scanner.c - GP7 digital input for the Code 39 barcode sensor.
 * The RP2040 GPIO is accessed directly for the micro T-Kernel build.
 * Timing the bars is bar_capture.c's job, sampled by ir_sense_task.
 */

#include <tk/tkernel.h>

#include "barcode_scanner.h"

#define BARCODE_IO_BANK0_BASE    0x40014000UL
#define BARCODE_PADS_BANK0_BASE  0x4001C000UL
#define BARCODE_SIO_BASE         0xD0000000UL
#define REG32(addr)      (*(volatile uint32_t *)(addr))
#define BARCODE_GPIO_CTRL(pin) REG32(BARCODE_IO_BANK0_BASE + 0x04UL + 8UL * (pin))
#define BARCODE_PAD_CTRL(pin)  REG32(BARCODE_PADS_BANK0_BASE + 0x04UL + 4UL * (pin))
#define BARCODE_SIO_GPIO_IN    REG32(BARCODE_SIO_BASE + 0x04UL)
#define BARCODE_SIO_GPIO_OE_CLR REG32(BARCODE_SIO_BASE + 0x28UL)

#define FUNCSEL_SIO      5u
#define PAD_IE           (1u << 6)
#define PAD_PUE          (1u << 3)
#define PAD_PDE          (1u << 2)

typedef struct {
	barcode_scanner_config_t config;
	uint16_t last_raw;
	bool black;
} barcode_scanner_state_t;

static barcode_scanner_state_t state;

static void input_pin_init(uint32_t pin)
{
	uint32_t pad;

	BARCODE_GPIO_CTRL(pin) = FUNCSEL_SIO;
	BARCODE_SIO_GPIO_OE_CLR = 1u << pin;
	pad = BARCODE_PAD_CTRL(pin);
	pad |= PAD_IE;
	pad &= ~(PAD_PUE | PAD_PDE);
	BARCODE_PAD_CTRL(pin) = pad;
}

barcode_scanner_config_t barcode_scanner_default_config(void)
{
	barcode_scanner_config_t config = {
		.black_is_high = (BARCODE_BLACK_IS_HIGH != 0u),
	};
	return config;
}

ER barcode_scanner_init(const barcode_scanner_config_t *config)
{
	state.config = (config != NULL) ? *config
					: barcode_scanner_default_config();
	state.last_raw = 0u;
	state.black = false;
	input_pin_init(BARCODE_SENSOR_GPIO);
	return E_OK;
}

uint16_t barcode_scanner_read_raw(void)
{
	return (uint16_t)((BARCODE_SIO_GPIO_IN >> BARCODE_SENSOR_GPIO) & 1u);
}

bool barcode_scanner_read_black(void)
{
	state.last_raw = barcode_scanner_read_raw();
	state.black = (state.last_raw != 0u) == state.config.black_is_high;
	return state.black;
}

uint16_t barcode_scanner_get_last_raw(void)
{
	return state.last_raw;
}
