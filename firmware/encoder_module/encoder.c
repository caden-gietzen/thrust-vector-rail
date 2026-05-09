#include "encoder.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"

#include "pico/time.h"

#define ENCODER_DEFAULT_MAX_STEP_RATE   0
#define ENCODER_DEFAULT_DRAIN_HZ        1000

static uint32_t s_max_step_rate = ENCODER_DEFAULT_MAX_STEP_RATE;
static uint32_t s_drain_hz = ENCODER_DEFAULT_DRAIN_HZ;

static void encoder_stop_drain_timer(void);
static bool encoder_start_drain_timer(void);

static PIO s_pio = pio0;
static uint s_sm = 0;
static bool s_loaded = false;
static bool s_initialized = false;
static uint s_pin_ab = 0;
static int32_t s_zero_offset = 0;

// drain tuning variables
static volatile int32_t s_latest_count = 0;
static volatile bool s_have_latest_count = false;

// Repeating timer
static struct repeating_timer s_drain_timer;
static bool s_timer_running = false;

/**/
void encoder_set_drain_rate_c(uint32_t drain_hz) {
    s_drain_hz = drain_hz;

    if (!s_initialized) {
        return;
    }

    encoder_stop_drain_timer();

    if (s_drain_hz > 0) {
        if (encoder_start_drain_timer()) {
            s_timer_running = true;
        }
    }
}

uint32_t encoder_get_drain_rate_c(void) {
    return s_drain_hz;
}

// Define timer callback:
static bool encoder_drain_timer_cb(struct repeating_timer *t) {
    if (!s_initialized) {
        return true;
    }

    int drained = 0;
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm) && drained < 64) {
        s_latest_count = (int32_t)pio_sm_get(s_pio, s_sm);
        s_have_latest_count = true;
        drained++;
    }

    return true;
}

/* helper to clear RX FIFO*/
static void encoder_clear_rx_fifo(void) {
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm)) {
        (void)pio_sm_get(s_pio, s_sm);
    }
}

/* timer restart helper */
static void encoder_stop_drain_timer(void) {
    if (s_timer_running) {
        cancel_repeating_timer(&s_drain_timer);
        s_timer_running = false;
    }
}

// helper to start timer from s_drain_hz
static bool encoder_start_drain_timer(void) {
    if (s_drain_hz == 0) {
        return false;
    }

    int64_t period_us = 1000000 / s_drain_hz;
    return add_repeating_timer_us(-period_us, encoder_drain_timer_cb, NULL, &s_drain_timer);
}

// load PIO program, initialize state machine, start drain timer
bool encoder_init_configured_c(int pin_a, int pin_b,
                               uint32_t max_step_rate,
                               uint32_t drain_hz) {
    
    s_zero_offset = 0;

    if (pin_b != pin_a + 1) {
        return false;
    }

    if (s_initialized) {
        encoder_deinit_c();
    }

    s_pin_ab = (uint)pin_a;
    s_max_step_rate = max_step_rate;
    s_drain_hz = drain_hz;

    s_latest_count = 0;
    s_have_latest_count = false;

    if (!s_loaded) {
        pio_add_program(s_pio, &quadrature_encoder_program);
        s_loaded = true;
    }

    quadrature_encoder_program_init(s_pio, s_sm, s_pin_ab, s_max_step_rate);

    encoder_clear_rx_fifo();
    sleep_ms(20);
    encoder_clear_rx_fifo();

    s_latest_count = 0;
    s_have_latest_count = true;

    s_initialized = true;

    if (!encoder_start_drain_timer()) {
        pio_sm_set_enabled(s_pio, s_sm, false);
        s_initialized = false;
        s_have_latest_count = false;
        return false;
    }

    s_timer_running = true;
    return true;
}

/* Convenience wrapper */
bool encoder_init_c(int pin_a, int pin_b) {
    return encoder_init_configured_c(pin_a, pin_b, 100000, 1000);
}

/*resets count by reinitializing the PIO program, also resets cache, clears stale FIFO*/


static void encoder_drain_once(void) {
    int drained = 0;
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm) && drained < 64) {
        s_latest_count = (int32_t)pio_sm_get(s_pio, s_sm);
        s_have_latest_count = true;
        drained++;
    }
}

void encoder_zero_c(void) {
    if (!s_initialized) {
        return;
    }

    // optional: one immediate drain/update so zero is based on freshest raw count
    encoder_drain_once();

    s_zero_offset = s_latest_count;
    s_have_latest_count = true;
}

int32_t encoder_get_count_c(void) {
    if (!s_have_latest_count) {
        encoder_drain_once();
    }
    return s_latest_count - s_zero_offset;
}

void encoder_deinit_c(void) {
    if (!s_initialized) {
        return;
    }

    s_zero_offset = 0;

    encoder_stop_drain_timer();
    pio_sm_set_enabled(s_pio, s_sm, false);
    encoder_clear_rx_fifo();

    s_initialized = false;
    s_have_latest_count = false;
    s_latest_count = 0;
}