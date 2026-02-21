#include "log.h"

#include <stdio.h>
#include <string.h>

#include "flash_w25q.h"
#include "uart.h"

#define LOG_BASE        0x000000u
#define LOG_SIZE        (16u * 1024u * 1024u) /* W25Q128 = 16MB */
#define LOG_SECTOR_SZ   4096u
#define LOG_PAGE_MAX    256u
#define LOG_REC_SZ      ((uint32_t)sizeof(log_rec_t))
#define LOG_PAGE_CHUNK  ((LOG_PAGE_MAX / LOG_REC_SZ) * LOG_REC_SZ)
#define LOG_MAX_RECS    (LOG_SIZE / LOG_REC_SZ)
#define LOG_RAM_RECS    128u

#if LOG_PAGE_CHUNK == 0
#error "log_rec_t is larger than flash page size"
#endif

static uint32_t g_wr = LOG_BASE;
static uint32_t g_seq = 0;
static uint32_t g_count = 0;
static uint32_t g_buf_used = 0;
static uint8_t g_buf[LOG_PAGE_CHUNK];

static uint8_t g_flash_ok = 0;
static uint32_t g_drop_count = 0;
static uint32_t g_flash_fail_count = 0;

static uint16_t g_ram_wr = 0;
static uint16_t g_ram_count = 0;
static log_rec_t g_ram[LOG_RAM_RECS];

static void uart_write_len(const char* s, int n, uint32_t cap) {
  if ((s == 0) || (n <= 0) || (cap == 0u)) {
    return;
  }
  uint32_t len = (uint32_t)n;
  if (len >= cap) {
    len = cap - 1u;
  }
  for (uint32_t i = 0; i < len; ++i) {
    uart2_putc(s[i]);
  }
}

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

static uint16_t ram_oldest_idx(void) {
  return (uint16_t)((g_ram_wr + LOG_RAM_RECS - g_ram_count) % LOG_RAM_RECS);
}

static int ram_peek_oldest(log_rec_t* out) {
  if ((out == 0) || (g_ram_count == 0u)) {
    return -1;
  }
  *out = g_ram[ram_oldest_idx()];
  return 0;
}

static void ram_pop_oldest(void) {
  if (g_ram_count > 0u) {
    --g_ram_count;
  }
}

static void ram_push(const log_rec_t* r) {
  if (g_ram_count < LOG_RAM_RECS) {
    g_ram_count++;
  } else {
    g_drop_count++;
  }
  g_ram[g_ram_wr] = *r;
  g_ram_wr++;
  if (g_ram_wr >= LOG_RAM_RECS) {
    g_ram_wr = 0;
  }
}

static void salvage_buffer_to_ram(void) {
  if (g_buf_used < LOG_REC_SZ) {
    g_buf_used = 0;
    return;
  }
  uint32_t off = 0;
  while ((off + LOG_REC_SZ) <= g_buf_used) {
    log_rec_t r = {0};
    memcpy(&r, &g_buf[off], LOG_REC_SZ);
    ram_push(&r);
    off += LOG_REC_SZ;
  }
  g_buf_used = 0;
}

static int flash_probe(void) {
  uint32_t id = 0;
  if (w25q_init() != 0) {
    return -1;
  }
  if (w25q_read_jedec_id(&id) != 0) {
    return -2;
  }
  return 0;
}

static int log_flush_chunk(const uint8_t* data, uint32_t len) {
  uint32_t off = 0;
  while (off < len) {
    log_wrap_if_needed();

    uint32_t sector_off = g_wr & (LOG_SECTOR_SZ - 1u);
    if (sector_off == 0u) {
      if (w25q_erase_4k(g_wr) != 0) {
        return -1;
      }
    }

    uint32_t remain_flash = (LOG_BASE + LOG_SIZE) - g_wr;
    uint32_t remain_sector = LOG_SECTOR_SZ - sector_off;
    uint32_t chunk = len - off;
    if (chunk > remain_sector) chunk = remain_sector;
    if (chunk > remain_flash) chunk = remain_flash;

    if (w25q_write(g_wr, data + off, chunk) != 0) {
      return -2;
    }
    g_wr += chunk;
    off += chunk;
  }
  log_wrap_if_needed();
  return 0;
}

static int log_flush_ram_to_flash(void) {
  while (g_ram_count > 0u) {
    log_rec_t r = {0};
    if (ram_peek_oldest(&r) != 0) {
      return -1;
    }
    if (log_flush_chunk((const uint8_t*)&r, LOG_REC_SZ) != 0) {
      return -2;
    }
    ram_pop_oldest();
    if (g_count < LOG_MAX_RECS) {
      ++g_count;
    }
  }
  return 0;
}

static void dump_rec_uart(const log_rec_t* r, uint16_t crc_ok) {
  char line[220];
  int n = snprintf(line, sizeof(line),
                   "%lu,%lu,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,0x%04X,%u\r\n",
                   (unsigned long)r->ms,
                   (unsigned long)r->seq,
                   (long)r->pressure_pa,
                   (long)r->altitude_cm,
                   r->temp_centi,
                   r->ax_mg, r->ay_mg, r->az_mg,
                   r->gx_dps_x10, r->gy_dps_x10, r->gz_dps_x10,
                   r->flags,
                   crc_ok);
  uart_write_len(line, n, sizeof(line));
}

static void dump_ram_records_uart(void) {
  if (g_ram_count == 0u) {
    return;
  }

  uint16_t start = (uint16_t)((g_ram_wr + LOG_RAM_RECS - g_ram_count) % LOG_RAM_RECS);
  for (uint16_t i = 0; i < g_ram_count; ++i) {
    uint16_t idx = (uint16_t)((start + i) % LOG_RAM_RECS);
    log_rec_t r = g_ram[idx];
    uint16_t crc_saved = r.crc;
    r.crc = 0;
    uint16_t ok = (crc_saved == crc16_ccitt((const uint8_t*)&r, LOG_REC_SZ)) ? 1u : 0u;
    dump_rec_uart(&g_ram[idx], ok);
  }
}

int log_init(void) {
  g_wr = LOG_BASE;
  g_seq = 0;
  g_count = 0;
  g_buf_used = 0;
  g_flash_ok = 0;
  g_drop_count = 0;
  g_flash_fail_count = 0;
  g_ram_wr = 0;
  g_ram_count = 0;

  if (flash_probe() != 0) {
    return -2;
  }

  /* Start fresh immediately; subsequent sectors are erased on-demand. */
  if (w25q_erase_4k(LOG_BASE) != 0) {
    return -3;
  }

  g_flash_ok = 1u;
  return 0;
}

void log_append(const log_rec_t* r_in) {
  if (!r_in) {
    return;
  }

  log_rec_t r = *r_in;
  r.seq = g_seq++;
  r.reserved = 0;
  r.crc = 0;
  r.crc = crc16_ccitt((const uint8_t*)&r, LOG_REC_SZ);

  /* Keep sequence ordering: while backlog exists, keep buffering in RAM. */
  if ((!g_flash_ok) || (g_ram_count > 0u)) {
    ram_push(&r);
    return;
  }

  if ((g_buf_used + LOG_REC_SZ) > LOG_PAGE_CHUNK) {
    log_flush();
  }

  if (!g_flash_ok) {
    ram_push(&r);
    return;
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
  if (!g_flash_ok) {
    if (flash_probe() == 0) {
      g_flash_ok = 1u;
    }
  }

  if (!g_flash_ok) {
    salvage_buffer_to_ram();
    return;
  }

  if (g_ram_count > 0u) {
    if (log_flush_ram_to_flash() != 0) {
      g_flash_fail_count++;
      g_flash_ok = 0u;
      salvage_buffer_to_ram();
      return;
    }
  }

  if (g_buf_used == 0u) {
    return;
  }

  if (log_flush_chunk(g_buf, g_buf_used) != 0) {
    g_flash_fail_count++;
    g_flash_ok = 0u;
    salvage_buffer_to_ram();
    return;
  }

  g_buf_used = 0;
}

void log_dump_uart(void) {
  log_flush();

  uart2_puts("DUMP_BEGIN\r\n");
  if (g_flash_ok && (g_ram_count == 0u)) {
    uart2_puts("#mode,flash\r\n");
  } else {
    uart2_puts("#mode,ram_fallback\r\n");
  }

  char meta[128];
  int m = snprintf(meta, sizeof(meta), "#dropped,%lu\r\n#flash_fail,%lu\r\n#ram_pending,%u\r\n",
                   (unsigned long)g_drop_count,
                   (unsigned long)g_flash_fail_count,
                   (unsigned)g_ram_count);
  uart_write_len(meta, m, sizeof(meta));

  uart2_puts("ms,seq,pressure_pa,altitude_cm,temp_centi,ax_mg,ay_mg,az_mg,gx_dps_x10,gy_dps_x10,gz_dps_x10,flags,crc_ok\r\n");

  if (g_flash_ok && (g_count > 0u)) {
    uint32_t addr = (g_count == LOG_MAX_RECS) ? g_wr : LOG_BASE;
    log_rec_t r;
    for (uint32_t i = 0; i < g_count; ++i) {
      if (w25q_read(addr, (uint8_t*)&r, LOG_REC_SZ) != 0) {
        g_flash_ok = 0u;
        g_flash_fail_count++;
        break;
      }

      uint16_t crc_saved = r.crc;
      r.crc = 0;
      uint16_t ok = (crc_saved == crc16_ccitt((const uint8_t*)&r, LOG_REC_SZ)) ? 1u : 0u;
      dump_rec_uart(&r, ok);

      addr += LOG_REC_SZ;
      if (addr >= (LOG_BASE + LOG_SIZE)) {
        addr = LOG_BASE;
      }
    }
  }

  /* If flash failed at runtime, keep recent samples available in RAM fallback. */
  dump_ram_records_uart();
  uart2_puts("DUMP_END\r\n");
}

int log_is_flash_enabled(void) {
  return g_flash_ok ? 1 : 0;
}

uint32_t log_dropped_records(void) {
  return g_drop_count;
}
