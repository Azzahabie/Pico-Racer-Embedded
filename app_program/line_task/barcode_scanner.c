/*
 * barcode_scanner.c - GP7 digital edge capture for Code 39 A-Z.
 * The RP2040 GPIO and timer are accessed directly for the micro T-Kernel build.
 */

#include <tk/tkernel.h>

#include "barcode_scanner.h"

#define BARCODE_TIMER_BASE       0x40054000UL
#define BARCODE_TIMER_RAW_HIGH   REG32(BARCODE_TIMER_BASE + 0x24UL)
#define BARCODE_TIMER_RAW_LOW    REG32(BARCODE_TIMER_BASE + 0x28UL)
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
#define TIMER_READ_LIMIT 8u

typedef struct {
	barcode_scanner_config_t config;
	uint16_t last_raw;
	bool black;
	bool scanning;
	bool last_edge_black;
	uint32_t minimum_width_us;
	uint64_t scan_started_us;
	uint64_t last_edge_us;
	uint32_t widths[BARCODE_MAX_WIDTHS];
	uint16_t width_count;
} barcode_scanner_state_t;

static barcode_scanner_state_t state;
static ID decoded_mutex;
static bool mutex_created;
static char decoded_text[BARCODE_MAX_CHARACTERS + 1u];
static ER last_error = E_OK;

static void copy_chars(char *destination, const char *source, uint16_t count)
{
	uint16_t i;

	for (i = 0u; i < count; ++i) {
		destination[i] = source[i];
	}
}

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

static uint64_t timer_now_us(void)
{
	uint32_t high_before = 0u;
	uint32_t high_after = 0u;
	uint32_t low = 0u;
	uint32_t attempt;

	for (attempt = 0u; attempt < TIMER_READ_LIMIT; ++attempt) {
		high_before = BARCODE_TIMER_RAW_HIGH;
		low = BARCODE_TIMER_RAW_LOW;
		high_after = BARCODE_TIMER_RAW_HIGH;
		if (high_before == high_after) {
			break;
		}
	}
	return ((uint64_t)high_after << 32) | low;
}

static void finish_scan(barcode_decode_result_t *result)
{
	barcode_decode_widths(state.widths, state.width_count, result);
	if (result->valid) {
		last_error = barcode_scanner_publish_decoded(result->text);
	}
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
	T_CMTX mutex_config = {
		.mtxatr = TA_TFIFO | TA_INHERIT,
	};
	ER result;

	state.config = (config != NULL) ? *config
					: barcode_scanner_default_config();
	state.last_raw = 0u;
	state.black = false;
	state.scanning = false;
	state.last_edge_black = false;
	state.minimum_width_us = UINT32_MAX;
	state.scan_started_us = 0u;
	state.last_edge_us = 0u;
	state.width_count = 0u;

	if (!mutex_created) {
		decoded_mutex = tk_cre_mtx(&mutex_config);
		if (decoded_mutex <= E_OK) {
			last_error = (ER)decoded_mutex;
			return last_error;
		}
		mutex_created = true;
	}

	result = barcode_scanner_publish_decoded("");
	if (result < E_OK) {
		last_error = result;
		return result;
	}
	input_pin_init(BARCODE_SENSOR_GPIO);
	last_error = E_OK;
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

bool barcode_scanner_scan_active(void)
{
	return state.scanning;
}

ER barcode_scanner_last_error(void)
{
	return last_error;
}

bool barcode_scanner_poll(barcode_decode_result_t *result)
{
	uint64_t now;
	bool black;

	if (result == NULL) {
		last_error = E_PAR;
		return false;
	}

	black = barcode_scanner_read_black();
	now = timer_now_us();
	if (!state.scanning) {
		if (black) {
			state.scanning = true;
			state.last_edge_black = true;
			state.minimum_width_us = UINT32_MAX;
			state.scan_started_us = now;
			state.last_edge_us = now;
			state.width_count = 0u;
		}
		return false;
	}

	if ((now - state.scan_started_us) >= BARCODE_SCAN_MAX_US) {
		result->valid = false;
		result->error = BARCODE_DECODE_TOO_LONG;
		result->error_character = 0u;
		state.scanning = false;
		return true;
	}

	if (black != state.last_edge_black) {
		uint64_t elapsed = now - state.last_edge_us;

		if (state.width_count >= BARCODE_MAX_WIDTHS) {
			result->valid = false;
			result->error = BARCODE_DECODE_TOO_LONG;
			result->error_character = 0u;
			state.scanning = false;
			return true;
		}
		state.widths[state.width_count++] =
			(elapsed > UINT32_MAX) ? UINT32_MAX : (uint32_t)elapsed;
		if (state.widths[state.width_count - 1u] <
		    state.minimum_width_us) {
			state.minimum_width_us = state.widths[state.width_count - 1u];
		}
		state.last_edge_us = now;
		state.last_edge_black = black;
		if (!black) {
			barcode_decode_result_t candidate;

			barcode_decode_widths(state.widths, state.width_count,
					      &candidate);
			if (candidate.valid) {
				last_error = barcode_scanner_publish_decoded(
					candidate.text);
				*result = candidate;
				state.scanning = false;
				return true;
			}
		}
		return false;
	}

	{
		uint64_t idle_timeout = BARCODE_SCAN_END_GAP_US;
		uint64_t width_timeout =
			(state.minimum_width_us == UINT32_MAX)
				? 0u
				: (uint64_t)state.minimum_width_us * 4u;

		if (width_timeout > idle_timeout) {
			idle_timeout = width_timeout;
		}
		if (!black && ((now - state.last_edge_us) >= idle_timeout)) {
			finish_scan(result);
			state.scanning = false;
			return true;
		}
	}
	return false;
}

ER barcode_scanner_publish_decoded(const char *text)
{
	char temporary[BARCODE_MAX_CHARACTERS + 1u];
	uint16_t length = 0u;
	ER result;

	if ((text == NULL) || !mutex_created) {
		return E_PAR;
	}
	while ((length < BARCODE_MAX_CHARACTERS) && (text[length] != '\0')) {
		temporary[length] = text[length];
		++length;
	}
	if (text[length] != '\0') {
		return E_PAR;
	}
	temporary[length] = '\0';

	result = tk_loc_mtx(decoded_mutex, TMO_FEVR);
	if (result < E_OK) {
		return result;
	}
	copy_chars(decoded_text, temporary, (uint16_t)(length + 1u));
	result = tk_unl_mtx(decoded_mutex);
	return result;
}

ER barcode_scanner_get_decoded(char *output, uint16_t output_size,
			       uint16_t *length)
{
	uint16_t current_length = 0u;
	ER result;

	if ((output == NULL) || (output_size == 0u) || (length == NULL) ||
	    !mutex_created) {
		return E_PAR;
	}

	result = tk_loc_mtx(decoded_mutex, TMO_FEVR);
	if (result < E_OK) {
		return result;
	}
	while ((current_length < BARCODE_MAX_CHARACTERS) &&
	       (decoded_text[current_length] != '\0')) {
		++current_length;
	}
	if (output_size <= current_length) {
		result = tk_unl_mtx(decoded_mutex);
		return (result < E_OK) ? result : E_PAR;
	}
	copy_chars(output, decoded_text, (uint16_t)(current_length + 1u));
	*length = current_length;
	return tk_unl_mtx(decoded_mutex);
}

uint16_t barcode_scanner_copy_decoded(char *output, uint16_t output_size)
{
	char temporary[BARCODE_MAX_CHARACTERS + 1u];
	uint16_t length = 0u;
	uint16_t copied;

	if (barcode_scanner_get_decoded(temporary, sizeof(temporary), &length) < E_OK) {
		return UINT16_MAX;
	}
	if ((output == NULL) || (output_size == 0u)) {
		return length;
	}
	copied = (length >= output_size) ? (uint16_t)(output_size - 1u) : length;
	copy_chars(output, temporary, copied);
	output[copied] = '\0';
	return copied;
}
