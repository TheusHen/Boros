#include "stm32g071.h"

void SystemInit(void){
    FLASH->ACR = 0;

    RCC->CR |= (1u << 8); 
    while (!(RCC->CR & (1u << 10))) { }

    RCC->CFGR &= ~0x3u;

    RCC->IOPENR |= (1u<<0) | (1u<<1) | (1u<<2);
    RCC->APBENR2 |= (1u<<12);  // SPI1EN (APBENR2)
    RCC->APBENR1 |= (1u<<17);  // USART2EN (APBENR1)
}