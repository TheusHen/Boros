#include "log.h"

#include <stdio.h>
#include <string.h>

#include "flash_w25q.h"
#include "uart.h"

#define LOG_BASE       0x000000u
#define LOG_SIZE       (16u * 1024u * 1024u) /* W25Q128 = 16MB */
#define LOG_SECTOR_SZ  4096u
#define LOG_PAGE_MAX   256u
#define LOG_REC_SZ     ((uint32_t)sizeof(log_rec_t))
#define LOG_PAGE_CHUNK ((LOG_PAGE_MAX / LOG_REC_SZ) * LOG_REC_SZ)
#define LOG_MAX_RECS   (LOG_SIZE / LOG_REC_SZ)

#if LOG_PAGE_CHUNK == 0
#error "log_rec_t is larger than flash page size"
#endif

static uint32_t g_wr = LOG_BASE;
static uint32_t g_seq = 0;
static uint32_t g_count = 0;
static uint32_t g_buf_used = 0;
static uint8_t g_buf[LOG_PAGE_CHUNK];

static uint16_t crc16_ccitt(const uint8_t* data, uint32_t len) {
  uint16_t crc = 0xFFFFu;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint32_t b = 0; b < 8; ++b) {
      if (crc & 0x8000u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static void log_wrap_if_needed(void) {
  if (g_wr >= (LOG_BASE + LOG_SIZE)) {
    g_wr = LOG_BASE;
  }
}

static void log_flush_chunk(const uint8_t* data, uint32_t len) {
  uint32_t off = 0;
  while (off < len) {
    log_wrap_if_needed();

    uint32_t sector_off = g_wr & (LOG_SECTOR_SZ - 1u);
    if (sector_off == 0u) {
      w25q_erase_4k(g_wr);
    }

    uint32_t remain_flash = (LOG_BASE + LOG_SIZE) - g_wr;
    uint32_t remain_sector = LOG_SECTOR_SZ - sector_off;
    uint32_t chunk = len - off;
    if (chunk > remain_sector) chunk = remain_sector;
    if (chunk > remain_flash) chunk = remain_flash;

    w25q_write(g_wr, data + off, chunk);
    g_wr += chunk;
    off += chunk;
  }
  log_wrap_if_needed();
}

void log_init(void) {
  w25q_init();
  g_wr = LOG_BASE;
  g_seq = 0;
  g_count = 0;
  g_buf_used = 0;

  /* Start fresh immediately; subsequent sectors are erased on-demand. */
  w25q_erase_4k(LOG_BASE);
}

void log_append(const log_rec_t* r_in) {
  log_rec_t r = *r_in;
  r.seq = g_seq++;
  r.reserved = 0;
  r.crc = 0;
  r.crc = crc16_ccitt((const uint8_t*)&r, LOG_REC_SZ);

  if ((g_buf_used + LOG_REC_SZ) > LOG_PAGE_CHUNK) {
    log_flush();
  }

  memcpy(&g_buf[g_buf_used], &r, LOG_REC_SZ);
  g_buf_used += LOG_REC_SZ;

  if (g_count < LOG_MAX_RECS) {
    ++g_count;
  }

  if (g_buf_used == LOG_PAGE_CHUNK) {
    log_flush();
  }
}

void log_flush(void) {
  if (g_buf_used == 0u) {
    return;
  }
  log_flush_chunk(g_buf, g_buf_used);
  g_buf_used = 0;
}

void log_dump_uart(void) {
  log_flush();

  uart2_puts("DUMP_BEGIN\r\n");
  uart2_puts("ms,seq,pressure_pa,altitude_cm,temp_centi,ax_mg,ay_mg,az_mg,gx_dps_x10,gy_dps_x10,gz_dps_x10,flags,crc_ok\r\n");

  if (g_count == 0u) {
    uart2_puts("DUMP_END\r\n");
    return;
  }

  uint32_t addr = (g_count == LOG_MAX_RECS) ? g_wr : LOG_BASE;
  log_rec_t r;
  for (uint32_t i = 0; i < g_count; ++i) {
    w25q_read(addr, (uint8_t*)&r, LOG_REC_SZ);

    uint16_t crc_saved = r.crc;
    r.crc = 0;
    uint16_t ok = (crc_saved == crc16_ccitt((const uint8_t*)&r, LOG_REC_SZ)) ? 1u : 0u;

    char line[220];
    int n = snprintf(line, sizeof(line),
                     "%lu,%lu,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,0x%04X,%u\r\n",
                     (unsigned long)r.ms,
                     (unsigned long)r.seq,
                     (long)r.pressure_pa,
                     (long)r.altitude_cm,
                     r.temp_centi,
                     r.ax_mg, r.ay_mg, r.az_mg,
                     r.gx_dps_x10, r.gy_dps_x10, r.gz_dps_x10,
                     r.flags,
                     ok);

    if (n > 0) {
      for (int k = 0; k < n; ++k) {
        uart2_putc(line[k]);
      }
    }

    addr += LOG_REC_SZ;
    if (addr >= (LOG_BASE + LOG_SIZE)) {
      addr = LOG_BASE;
    }
  }

  uart2_puts("DUMP_END\r\n");
}
