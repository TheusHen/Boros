#pragma once
#include <stdint.h>

void uart2_init(uint32_t cpu_hz, uint32_t baud);
void uart2_putc(char c);
void uart2_puts(const char* s);
int  uart2_getc_nonblock(void);