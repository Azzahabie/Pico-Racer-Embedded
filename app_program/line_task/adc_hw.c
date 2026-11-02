#include "adc_hw.h"

#include <stdbool.h>

#define ADC_IO_BANK0_BASE    0x40014000UL
#define ADC_PADS_BANK0_BASE  0x4001C000UL
#define ADC_RESETS_BASE      0x4000C000UL
#define ADC_CLOCKS_BASE      0x40008000UL
#define ADC_BASE             0x4004C000UL

#define REG32(addr)          (*(volatile uint32_t *)(addr))
#define ADC_GPIO_CTRL(pin)   REG32(ADC_IO_BANK0_BASE + 0x04UL + 8UL * (pin))
#define ADC_PAD_CTRL_CLR(pin) REG32(ADC_PADS_BANK0_BASE + 0x3000UL + 0x04UL + 4UL * (pin))
#define ADC_RESET_CLR        REG32(ADC_RESETS_BASE + 0x3000UL)
#define ADC_RESET_DONE       REG32(ADC_RESETS_BASE + 0x08UL)
#define ADC_CLK_CTRL         REG32(ADC_CLOCKS_BASE + 0x60UL)
#define ADC_CS               REG32(ADC_BASE + 0x00UL)
#define ADC_RESULT           REG32(ADC_BASE + 0x04UL)

#define RESET_ADC_BIT        (1u << 0)
#define CLK_ADC_ENABLE       (1u << 11)
#define FUNCSEL_NULL         0x1Fu
#define PAD_IE               (1u << 6)
#define PAD_PUE              (1u << 3)
#define PAD_PDE              (1u << 2)
#define ADC_CS_EN            (1u << 0)
#define ADC_CS_START_ONCE    (1u << 2)
#define ADC_CS_READY         (1u << 8)
#define ADC_CS_AINSEL_SHIFT  12u
#define ADC_WAIT_LIMIT       200000UL
#define ADC_AVERAGE_SAMPLES  4u

static bool adc_initialized;
static bool previous_channel_valid;
static uint32_t previous_channel;
static ER last_error = E_OK;

static bool wait_set(volatile uint32_t *reg, uint32_t mask)
{
	uint32_t limit = ADC_WAIT_LIMIT;

	while (((*reg) & mask) == 0u) {
		if (limit == 0u) {
			return false;
		}
		--limit;
	}
	return true;
}

static ER adc_convert(uint32_t channel, uint16_t *value)
{
	ADC_CS = ADC_CS_EN | ADC_CS_START_ONCE |
		 (channel << ADC_CS_AINSEL_SHIFT);
	if (!wait_set(&ADC_CS, ADC_CS_READY)) {
		return E_TMOUT;
	}
	*value = (uint16_t)(ADC_RESULT & 0x0FFFu);
	return E_OK;
}

ER adc_hw_init(void)
{
	ER result;

	if (adc_initialized) {
		return E_OK;
	}

	ADC_RESET_CLR = RESET_ADC_BIT;
	if (!wait_set(&ADC_RESET_DONE, RESET_ADC_BIT)) {
		last_error = E_TMOUT;
		return last_error;
	}

	if ((ADC_CLK_CTRL & CLK_ADC_ENABLE) == 0u) {
		ADC_CLK_CTRL = CLK_ADC_ENABLE;
	}
	ADC_CS = ADC_CS_EN;
	if (!wait_set(&ADC_CS, ADC_CS_READY)) {
		last_error = E_TMOUT;
		return last_error;
	}

	result = E_OK;
	adc_initialized = true;
	previous_channel_valid = false;
	last_error = result;
	return result;
}

void adc_hw_pin_init(uint32_t gpio)
{
	ADC_GPIO_CTRL(gpio) = FUNCSEL_NULL;
	ADC_PAD_CTRL_CLR(gpio) = PAD_IE | PAD_PUE | PAD_PDE;
}

uint16_t adc_hw_read(uint32_t channel)
{
	uint16_t value;
	uint32_t total = 0u;
	uint32_t sample;
	ER result;

	if (!adc_initialized) {
		last_error = E_SYS;
		return UINT16_MAX;
	}
	if (channel > 4u) {
		last_error = E_PAR;
		return UINT16_MAX;
	}

	if (!previous_channel_valid || (channel != previous_channel)) {
		result = adc_convert(channel, &value);
		if (result < E_OK) {
			last_error = result;
			return UINT16_MAX;
		}
		previous_channel = channel;
		previous_channel_valid = true;
	}

	for (sample = 0u; sample < ADC_AVERAGE_SAMPLES; ++sample) {
		result = adc_convert(channel, &value);
		if (result < E_OK) {
			last_error = result;
			return UINT16_MAX;
		}
		total += value;
	}

	last_error = E_OK;
	return (uint16_t)(total / ADC_AVERAGE_SAMPLES);
}

ER adc_hw_last_error(void)
{
	return last_error;
}
