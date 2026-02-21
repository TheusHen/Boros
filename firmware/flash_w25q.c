#include "flash_w25q.h"
#include "spi.h"
#include "gpio.h"

#define CMD_WREN   0x06
#define CMD_RDSR1  0x05
#define CMD_READ   0x03
#define CMD_PP     0x02
#define CMD_SE_4K  0x20
#define CMD_RDID   0x9F

static uint8_t rdsr1(void) {
  uint8_t tx[2] = {CMD_RDSR1, 0xFF}, rx[2]={0};
  cs_flash(0); spi1_txrx(tx, rx, 2); cs_flash(1);
  return rx[1];
}

static void wait_ready(void) {
  while (rdsr1() & 0x01) { }
}

static void wren(void) {
  uint8_t cmd = CMD_WREN;
  cs_flash(0); spi1_txrx(&cmd, 0, 1); cs_flash(1);
}

void w25q_init(void) {
  wait_ready();
}

uint32_t w25q_read_jedec_id(void) {
  uint8_t tx[4] = {CMD_RDID, 0xFF, 0xFF, 0xFF};
  uint8_t rx[4] = {0};
  cs_flash(0);
  spi1_txrx(tx, rx, 4);
  cs_flash(1);
  return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}

void w25q_read(uint32_t addr, uint8_t* buf, uint32_t n) {
  uint8_t hdr[4] = {CMD_READ, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  cs_flash(0);
  spi1_txrx(hdr, 0, 4);
  spi1_txrx(0, buf, n);
  cs_flash(1);
}

static void page_program(uint32_t addr, const uint8_t* buf, uint32_t n) {
  wren();
  uint8_t hdr[4] = {CMD_PP, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  cs_flash(0);
  spi1_txrx(hdr, 0, 4);
  spi1_txrx(buf, 0, n);
  cs_flash(1);
  wait_ready();
}

void w25q_write(uint32_t addr, const uint8_t* buf, uint32_t n) {
  // split across 256B pages
  while (n) {
    uint32_t page_off = addr & 0xFFu;
    uint32_t chunk = 256u - page_off;
    if (chunk > n) chunk = n;
    page_program(addr, buf, chunk);
    addr += chunk; buf += chunk; n -= chunk;
  }
}

void w25q_erase_4k(uint32_t addr) {
  addr &= ~0xFFFu;
  wren();
  uint8_t cmd[4] = {CMD_SE_4K, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  cs_flash(0); spi1_txrx(cmd, 0, 4); cs_flash(1);
  wait_ready();
}
