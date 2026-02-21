#pragma once
#include <stdint.h>

void systick_init_1ms(uint32_t cpu_hz);
uint32_t millis(void);