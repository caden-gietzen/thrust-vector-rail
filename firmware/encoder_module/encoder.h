#pragma once

#include <stdbool.h>
#include <stdint.h>

bool encoder_init_c(int pin_a, int pin_b);
bool encoder_init_configured_c(int pin_a, int pin_b,
                               uint32_t max_step_rate,
                               uint32_t drain_hz);
int32_t encoder_get_count_c(void);
void encoder_zero_c(void);
void encoder_deinit_c(void);
void encoder_set_drain_rate_c(uint32_t drain_hz);
uint32_t encoder_get_drain_rate_c(void);