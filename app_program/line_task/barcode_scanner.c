/*
 * barcode_scanner.c - single analogue barcode sensor on GP26 (ADC0)
 *
 * Works in the micro T-Kernel build: it does NOT use the Pico SDK
 * (pico/stdlib.h and hardware/adc.h are not on this build's include path).
 * The ADC and the pin are driven directly through the RP2040 registers.
 *
 * Every wait on the hardware has a limit, so a stuck ADC returns a reading
 * instead of freezing the task.
 */

#include <stddef.h>

#include "barcode_scanner.h"

#define BARCODE_SCANNER_ADC_SAMPLES (8u)

/* ------------------------ RP2040 register access ----------------------- */

#define REG32(addr)         (*(volatile uint32_t *)(addr))

/* Atomic alias offsets: +0x2000 sets bits, +0x3000 clears bits. */
#define ALIAS_CLR           0x3000UL

#define RESETS_BASE         0x4000C000UL
#define RESETS_RESET_CLR    REG32(RESETS_BASE + ALIAS_CLR + 0x00UL)
#define RESETS_RESET_DONE   REG32(RESETS_BASE + 0x08UL)
#define RESET_ADC_BIT       (1u << 0)

#define CLOCKS_BASE         0x40008000UL
#define CLK_ADC_CTRL        REG32(CLOCKS_BASE + 0x60UL)
#define CLK_ADC_ENABLE      (1u << 11)

#define IO_BANK0_BASE       0x40014000UL
#define PADS_BANK0_BASE     0x4001C000UL
#define GPIO_CTRL(pin)      REG32(IO_BANK0_BASE + 0x04UL + 8UL * (pin))
#define PAD_CTRL_CLR(pin)   REG32(PADS_BANK0_BASE + ALIAS_CLR + 0x04UL + 4UL * (pin))
#define FUNCSEL_NULL        0x1Fu
#define PAD_IE              (1u << 6)   /* digital input enable */
#define PAD_PUE             (1u << 3)   /* pull-up              */
#define PAD_PDE             (1u << 2)   /* pull-down            */

#define ADC_BASE            0x4004C000UL
#define ADC_CS              REG32(ADC_BASE + 0x00UL)
#define ADC_RESULT          REG32(ADC_BASE + 0x04UL)
#define ADC_CS_EN           (1u << 0)
#define ADC_CS_START_ONCE   (1u << 2)
#define ADC_CS_READY        (1u << 8)
#define ADC_CS_AINSEL_SHIFT 12u

#define HW_WAIT_LIMIT       200000UL    /* loop count before giving up */

typedef struct {
    barcode_scanner_config_t config;
    uint16_t last_raw;
    bool black;
} barcode_scanner_state_t;

static barcode_scanner_state_t state;

/** Waits until (reg & mask) != 0 or the limit runs out. Returns true on success. */
static bool wait_set(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t guard = HW_WAIT_LIMIT;

    while (((*reg) & mask) == 0u) {
        if (guard-- == 0u) {
            return false;
        }
    }
    return true;
}

/** Takes the ADC out of reset, makes sure its clock runs, and enables it. */
static void adc_hw_init(void)
{
    RESETS_RESET_CLR = RESET_ADC_BIT;
    (void)wait_set(&RESETS_RESET_DONE, RESET_ADC_BIT);

    /* The ADC needs its own 48 MHz clock. It is normally already running
       from the USB PLL; only start it if the BSP left it off. */
    if ((CLK_ADC_CTRL & CLK_ADC_ENABLE) == 0u) {
        CLK_ADC_CTRL = CLK_ADC_ENABLE;      /* AUXSRC = 0: USB PLL */
    }

    ADC_CS = ADC_CS_EN;
    (void)wait_set(&ADC_CS, ADC_CS_READY);
}

/** Makes GP26 an analogue input: no function, no digital input, no pulls. */
static void adc_pin_init(uint32_t pin)
{
    GPIO_CTRL(pin) = FUNCSEL_NULL;
    PAD_CTRL_CLR(pin) = (PAD_IE | PAD_PUE | PAD_PDE);
}

/** One conversion on the sensor's channel. */
static uint16_t adc_read_once(void)
{
    ADC_CS = ADC_CS_EN | ADC_CS_START_ONCE |
             ((uint32_t)BARCODE_SCANNER_ADC_CHANNEL << ADC_CS_AINSEL_SHIFT);

    (void)wait_set(&ADC_CS, ADC_CS_READY);

    return (uint16_t)(ADC_RESULT & 0x0FFFu);
}

/** Reads the ADC repeatedly and averages the samples to reduce noise. */
static uint16_t read_average(void)
{
    uint32_t total = 0u;

    /* The first result may still contain charge from the previous channel. */
    (void)adc_read_once();

    for (uint32_t sample = 0u; sample < BARCODE_SCANNER_ADC_SAMPLES; ++sample) {
        total += adc_read_once();
    }

    return (uint16_t)(total / BARCODE_SCANNER_ADC_SAMPLES);
}

/**
 * Classifies a reading with different switching points for entering and leaving
 * the black state, preventing noisy values near the threshold from flickering.
 */
static bool detect_black(uint16_t reading,
                         uint16_t threshold,
                         uint16_t hysteresis,
                         bool black_is_high,
                         bool was_black)
{
    uint16_t lower = (threshold > hysteresis)
                         ? (uint16_t)(threshold - hysteresis)
                         : 0u;
    uint32_t upper_sum = (uint32_t)threshold + hysteresis;
    uint16_t upper = (upper_sum < 4095u) ? (uint16_t)upper_sum : 4095u;

    if (black_is_high) {
        return was_black ? (reading >= lower) : (reading >= upper);
    }

    return was_black ? (reading <= upper) : (reading <= lower);
}

/* ------------------------------- public ------------------------------- */

barcode_scanner_config_t barcode_scanner_default_config(void)
{
    barcode_scanner_config_t config = {
        .threshold = 2048u,
        .hysteresis = 64u,
        .black_is_high = true,
    };

    return config;
}

void barcode_scanner_init(const barcode_scanner_config_t *config)
{
    state.config =
        (config != NULL) ? *config : barcode_scanner_default_config();
    state.black = false;
    state.last_raw = 0u;

    adc_hw_init();
    adc_pin_init(BARCODE_SCANNER_GPIO);
}

uint16_t barcode_scanner_read_raw(void)
{
    return read_average();
}

bool barcode_scanner_read_black(void)
{
    uint16_t reading = barcode_scanner_read_raw();
    state.last_raw = reading;

    state.black = detect_black(
        reading,
        state.config.threshold,
        state.config.hysteresis,
        state.config.black_is_high,
        state.black);

    return state.black;
}

uint16_t barcode_scanner_get_last_raw(void)
{
    return state.last_raw;
}