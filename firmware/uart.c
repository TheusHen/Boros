#include "stm32g071.h"
#include "uart.h"

void uart2_init(uint32_t cpu_hz, uint32_t baud) {
  // 8N1, oversampling 16
  USART2->CR1 = 0;
  USART2->BRR = (cpu_hz + (baud/2)) / baud;
  USART2->CR1 = (1u<<3) | (1u<<2) | (1u<<0); // TE RE UE
}

void uart2_putc(char c) {
  while (!(USART2->ISR & (1u<<7))) { } // TXE
  USART2->TDR = (uint8_t)c;
}

void uart2_puts(const char* s) {
  while (*s) uart2_putc(*s++);
}

int uart2_getc_nonblock(void) {
  if (USART2->ISR & (1u<<5)) { // RXNE
    return (int)(USART2->RDR & 0xFF);
  }
  return -1;
}