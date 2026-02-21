#pragma once
#include <stdint.h>
void spi1_init(void);
uint8_t spi1_xfer(uint8_t b);
void spi1_txrx(const uint8_t* tx, uint8_t* rx, uint32_t n);