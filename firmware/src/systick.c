#include "stm32g071.h"
#include "systick.h"

static volatile uint32_t g_ms = 0;

void SysTick_Handler(void){
    g_ms++;
}

uint32_t millis(void) {return g_ms;}

void systick_init_1ms(uint32_t cpu_hz){
    SysTick->LOAD = cpu_hz / 1000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = (1u<<2) | (1u<<1) | (1u<<0);
}