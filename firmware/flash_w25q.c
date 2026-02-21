#include "flash_w25q.h"
#include "spi.h"
#include "gpio.h"

#define CMD_WREN   0x06
#define CMD_RDSR1  0x05
#define CMD_READ   0x03
#define CMD_PP     0x02
#define CMD_SE_4K  0x20
#define CMD_RDID   0x9F

#define W25Q_MAX_WAIT_POLLS   300000u
#define W25Q_MAX_WEL_POLLS    10000u

static uint8_t rdsr1(void) {
  uint8_t tx[2] = {CMD_RDSR1, 0xFF}, rx[2]={0};
  cs_flash(0); spi1_txrx(tx, rx, 2); cs_flash(1);
  return rx[1];
}

static int wait_ready(void) {
  for (uint32_t i = 0; i < W25Q_MAX_WAIT_POLLS; ++i) {
    if ((rdsr1() & 0x01u) == 0u) {
      return 0;
    }
  }
  return -1;
}

static int wren(void) {
  uint8_t cmd = CMD_WREN;
  cs_flash(0); spi1_txrx(&cmd, 0, 1); cs_flash(1);
  for (uint32_t i = 0; i < W25Q_MAX_WEL_POLLS; ++i) {
    if (rdsr1() & 0x02u) {
      return 0;
    }
  }
  return -1;
}

int w25q_init(void) {
  return wait_ready();
}

int w25q_read_jedec_id(uint32_t* jedec_id) {
  if (!jedec_id) {
    return -1;
  }
  uint8_t tx[4] = {CMD_RDID, 0xFF, 0xFF, 0xFF};
  uint8_t rx[4] = {0};
  cs_flash(0);
  spi1_txrx(tx, rx, 4);
  cs_flash(1);
  uint32_t id = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
  if ((id == 0u) || (id == 0xFFFFFFu)) {
    return -2;
  }
  *jedec_id = id;
  return 0;
}

int w25q_read(uint32_t addr, uint8_t* buf, uint32_t n) {
  if ((buf == 0) && (n != 0u)) {
    return -1;
  }
  uint8_t hdr[4] = {CMD_READ, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  cs_flash(0);
  spi1_txrx(hdr, 0, 4);
  spi1_txrx(0, buf, n);
  cs_flash(1);
  return 0;
}

static int page_program(uint32_t addr, const uint8_t* buf, uint32_t n) {
  if ((n == 0u) || (n > 256u)) {
    return -1;
  }
  if (wren() != 0) {
    return -2;
  }
  uint8_t hdr[4] = {CMD_PP, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  cs_flash(0);
  spi1_txrx(hdr, 0, 4);
  spi1_txrx(buf, 0, n);
  cs_flash(1);
  if (wait_ready() != 0) {
    return -3;
  }
  return 0;
}

int w25q_write(uint32_t addr, const uint8_t* buf, uint32_t n) {
  if ((buf == 0) && (n != 0u)) {
    return -1;
  }
  // split across 256B pages
  while (n) {
    uint32_t page_off = addr & 0xFFu;
    uint32_t chunk = 256u - page_off;
    if (chunk > n) chunk = n;
    if (page_program(addr, buf, chunk) != 0) {
      return -2;
    }
    addr += chunk; buf += chunk; n -= chunk;
  }
  return 0;
}

int w25q_erase_4k(uint32_t addr) {
  addr &= ~0xFFFu;
  if (wren() != 0) {
    return -1;
  }
  uint8_t cmd[4] = {CMD_SE_4K, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  cs_flash(0); spi1_txrx(cmd, 0, 4); cs_flash(1);
  if (wait_ready() != 0) {
    return -2;
  }
  return 0;
}
