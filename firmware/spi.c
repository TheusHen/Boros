#include "stm32g071.h"
#include "spi.h"

void spi1_init(void) {
  SPI1->CR1 = 0;
  // Master, BR= /16, CPOL=0 CPHA=0, SSM=1 SSI=1, SPE=1
  SPI1->CR1 = (1u<<2) | (3u<<3) | (1u<<9) | (1u<<8);
  SPI1->CR2 = (7u<<8);
  SPI1->CR1 |= (1u<<6);
}

uint8_t spi1_xfer(uint8_t b) {
  while (!(SPI1->SR & (1u<<1))) { } // TXE
  SPI1->DR = b;
  while (!(SPI1->SR & (1u<<0))) { } // RXNE
  return (uint8_t)SPI1->DR;
}

void spi1_txrx(const uint8_t* tx, uint8_t* rx, uint32_t n) {
  for (uint32_t i=0;i<n;i++) {
    uint8_t out = tx ? tx[i] : 0xFF;
    uint8_t in  = spi1_xfer(out);
    if (rx) rx[i] = in;
  }
}