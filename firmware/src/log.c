#include "log.h"

#include <stdio.h>
#include <string.h>

#include "flash_w25q.h"
#include "fault.h"
#include "uart.h"

#define LOG_BASE        0x000000u
#define LOG_SIZE        (16u * 1024u * 1024u) /* W25Q128 = 16MB */
#define LOG_SECTOR_SZ   4096u
#define LOG_PAGE_MAX    256u
#define LOG_RAM_RECS    256u

#define LOG_PKT_KEY     0xA1u
#define LOG_PKT_DELTA   0xA2u
#define LOG_KEY_INTERVAL 32u
#define LOG_MAX_PAYLOAD 220u
#define LOG_MAX_PACKET  (2u + LOG_MAX_PAYLOAD + 2u)

typedef char log_payload_guard[(sizeof(log_rec_t) <= LOG_MAX_PAYLOAD) ? 1 : -1];

static uint32_t g_wr = LOG_BASE;
static uint32_t g_used_bytes = 0;
static uint32_t g_seq = 0;
static uint32_t g_since_key = 0;
static uint32_t g_records_total = 0;

static uint32_t g_buf_used = 0;
static uint8_t g_buf[LOG_PAGE_MAX];

static uint8_t g_flash_ok = 0;
static uint8_t g_force_key = 1;
static uint8_t g_need_key = 1;
static uint8_t g_have_prev = 0;
static log_rec_t g_prev = {0};

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

static uint32_t zigzag_encode(int32_t v) {
  return ((uint32_t)v << 1) ^ ((uint32_t)(v >> 31));
}

static int32_t zigzag_decode(uint32_t v) {
  return (int32_t)((v >> 1) ^ (uint32_t)(-(int32_t)(v & 1u)));
}

static uint32_t put_uvar(uint8_t* dst, uint32_t cap, uint32_t v) {
  uint32_t n = 0;
  do {
    if (n >= cap) {
      return 0;
    }
    uint8_t b = (uint8_t)(v & 0x7Fu);
    v >>= 7;
    if (v != 0u) {
      b |= 0x80u;
    }
    dst[n++] = b;
  } while (v != 0u);
  return n;
}

static uint32_t put_svar(uint8_t* dst, uint32_t cap, int32_t v) {
  return put_uvar(dst, cap, zigzag_encode(v));
}

static int get_uvar(const uint8_t* src, uint32_t len, uint32_t* off, uint32_t* out) {
  uint32_t val = 0;
  uint32_t shift = 0;
  while ((*off) < len && shift < 35u) {
    uint8_t b = src[(*off)++];
    val |= ((uint32_t)(b & 0x7Fu) << shift);
    if ((b & 0x80u) == 0u) {
      *out = val;
      return 0;
    }
    shift += 7u;
  }
  return -1;
}

static int get_svar(const uint8_t* src, uint32_t len, uint32_t* off, int32_t* out) {
  uint32_t zz = 0;
  if (get_uvar(src, len, off, &zz) != 0) {
    return -1;
  }
  *out = zigzag_decode(zz);
  return 0;
}

static uint16_t clamp_u16(int32_t v) {
  if (v < 0) {
    return 0;
  }
  if (v > 65535) {
    return 65535;
  }
  return (uint16_t)v;
}

static uint16_t ram_oldest_idx(void) {
  return (uint16_t)((g_ram_wr + LOG_RAM_RECS - g_ram_count) % LOG_RAM_RECS);
}

static uint16_t ram_newest_idx(void) {
  return (uint16_t)((g_ram_wr + LOG_RAM_RECS - 1u) % LOG_RAM_RECS);
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
  if (r == 0) {
    return;
  }

  if (g_ram_count > 0u && g_ram[ram_newest_idx()].seq == r->seq) {
    return;
  }

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

static void addr_advance(uint32_t* addr, uint32_t n) {
  uint32_t end = LOG_BASE + LOG_SIZE;
  uint32_t v = *addr + n;
  while (v >= end) {
    v -= LOG_SIZE;
  }
  *addr = v;
}

static void account_written(uint32_t n) {
  if (g_used_bytes < LOG_SIZE) {
    uint32_t room = LOG_SIZE - g_used_bytes;
    g_used_bytes += (n > room) ? room : n;
  }
}

static int flash_read_wrap(uint32_t addr, uint8_t* dst, uint32_t n) {
  uint32_t end = LOG_BASE + LOG_SIZE;
  uint32_t first = n;
  if ((addr + n) > end) {
    first = end - addr;
  }
  if (first > 0u) {
    if (w25q_read(addr, dst, first) != 0) {
      return -1;
    }
  }
  if (first < n) {
    if (w25q_read(LOG_BASE, dst + first, n - first) != 0) {
      return -2;
    }
  }
  return 0;
}

static int log_flush_chunk(const uint8_t* data, uint32_t len) {
  uint32_t off = 0;
  while (off < len) {
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
    if (g_wr >= (LOG_BASE + LOG_SIZE)) {
      g_wr = LOG_BASE;
    }
    account_written(chunk);
    off += chunk;
  }
  return 0;
}

static int flush_buffer_to_flash(void) {
  if (g_buf_used == 0u) {
    return 0;
  }
  if (!g_flash_ok) {
    return -1;
  }
  if (log_flush_chunk(g_buf, g_buf_used) != 0) {
    return -2;
  }
  g_buf_used = 0;
  g_need_key = 1;
  return 0;
}

static int encode_key_packet(const log_rec_t* r, uint8_t* out, uint32_t cap, uint32_t* out_len) {
  uint32_t payload_len = (uint32_t)sizeof(log_rec_t);
  uint32_t total = 2u + payload_len + 2u;
  if ((payload_len > 255u) || (total > cap)) {
    return -1;
  }
  out[0] = LOG_PKT_KEY;
  out[1] = (uint8_t)payload_len;
  memcpy(&out[2], r, payload_len);
  uint16_t crc = crc16_ccitt(out, 2u + payload_len);
  out[2u + payload_len] = (uint8_t)(crc & 0xFFu);
  out[3u + payload_len] = (uint8_t)(crc >> 8);
  *out_len = total;
  return 0;
}

static int encode_delta_payload(const log_rec_t* prev, const log_rec_t* cur, uint8_t* out, uint32_t cap, uint32_t* out_len) {
  uint32_t off = 0;
  uint32_t n = 0;

  if ((cur->seq < prev->seq) || (cur->ms < prev->ms)) {
    return -1;
  }

#define PUT_U(VAR) \
  do { \
    n = put_uvar(out + off, cap - off, (uint32_t)(VAR)); \
    if (n == 0u) return -1; \
    off += n; \
  } while (0)

#define PUT_S(VAR) \
  do { \
    n = put_svar(out + off, cap - off, (int32_t)(VAR)); \
    if (n == 0u) return -1; \
    off += n; \
  } while (0)

  PUT_U(cur->seq - prev->seq);
  PUT_U(cur->ms - prev->ms);

  PUT_S(cur->pressure_pa - prev->pressure_pa);
  PUT_S(cur->altitude_cm - prev->altitude_cm);
  PUT_S((int32_t)cur->temp_centi - (int32_t)prev->temp_centi);

  PUT_S((int32_t)cur->ax_mg - (int32_t)prev->ax_mg);
  PUT_S((int32_t)cur->ay_mg - (int32_t)prev->ay_mg);
  PUT_S((int32_t)cur->az_mg - (int32_t)prev->az_mg);

  PUT_S((int32_t)cur->gx_dps_x10 - (int32_t)prev->gx_dps_x10);
  PUT_S((int32_t)cur->gy_dps_x10 - (int32_t)prev->gy_dps_x10);
  PUT_S((int32_t)cur->gz_dps_x10 - (int32_t)prev->gz_dps_x10);

  PUT_S((int32_t)cur->vz_cms - (int32_t)prev->vz_cms);
  PUT_S((int32_t)cur->vbat_mv - (int32_t)prev->vbat_mv);
  PUT_S((int32_t)cur->bmp_stale - (int32_t)prev->bmp_stale);
  PUT_S((int32_t)cur->imu_stale - (int32_t)prev->imu_stale);

  PUT_U(cur->flags ^ prev->flags);
  PUT_U(cur->flight_state);
  PUT_U(cur->reset_cause);

#undef PUT_U
#undef PUT_S

  *out_len = off;
  return 0;
}

static int encode_record_packet(const log_rec_t* r, uint8_t* out, uint32_t cap, uint32_t* out_len) {
  uint8_t payload[LOG_MAX_PAYLOAD];
  uint32_t payload_len = 0;
  uint32_t total = 0;
  int use_key = 0;

  if ((!g_have_prev) || g_force_key || g_need_key || (g_since_key >= LOG_KEY_INTERVAL)) {
    use_key = 1;
  }

  if (!use_key) {
    if (encode_delta_payload(&g_prev, r, payload, sizeof(payload), &payload_len) != 0) {
      use_key = 1;
    } else if (payload_len >= sizeof(log_rec_t)) {
      use_key = 1;
    }
  }

  if (use_key) {
    if (encode_key_packet(r, out, cap, out_len) != 0) {
      return -1;
    }
    g_since_key = 0;
  } else {
    if ((payload_len > 255u) || ((2u + payload_len + 2u) > cap)) {
      return -1;
    }
    out[0] = LOG_PKT_DELTA;
    out[1] = (uint8_t)payload_len;
    memcpy(&out[2], payload, payload_len);
    total = 2u + payload_len;
    {
      uint16_t crc = crc16_ccitt(out, total);
      out[total] = (uint8_t)(crc & 0xFFu);
      out[total + 1u] = (uint8_t)(crc >> 8);
    }
    *out_len = total + 2u;
    g_since_key++;
  }

  g_prev = *r;
  g_have_prev = 1;
  g_force_key = 0;
  g_need_key = 0;
  return 0;
}

static int append_packet(const uint8_t* pkt, uint32_t len, int* copied) {
  *copied = 0;
  if (len > sizeof(g_buf)) {
    return -1;
  }

  if ((g_buf_used + len) > sizeof(g_buf)) {
    if (flush_buffer_to_flash() != 0) {
      return -1;
    }
  }

  memcpy(&g_buf[g_buf_used], pkt, len);
  g_buf_used += len;
  *copied = 1;

  if (g_buf_used == sizeof(g_buf)) {
    if (flush_buffer_to_flash() != 0) {
      return -1;
    }
  }

  return 0;
}

static int decode_packet_payload(uint8_t tag, const uint8_t* payload, uint32_t len, log_rec_t* out, log_rec_t* prev, int* have_prev) {
  uint32_t off = 0;
  uint32_t uv = 0;
  int32_t sv = 0;

  if (tag == LOG_PKT_KEY) {
    if (len != sizeof(log_rec_t)) {
      return -1;
    }
    memcpy(out, payload, sizeof(log_rec_t));
    *prev = *out;
    *have_prev = 1;
    return 0;
  }

  if (tag != LOG_PKT_DELTA || !(*have_prev)) {
    return -1;
  }

  *out = *prev;

  if (get_uvar(payload, len, &off, &uv) != 0) return -1;
  out->seq += uv;
  if (get_uvar(payload, len, &off, &uv) != 0) return -1;
  out->ms += uv;

  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->pressure_pa += sv;
  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->altitude_cm += sv;
  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->temp_centi = (int16_t)((int32_t)out->temp_centi + sv);

  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->ax_mg = (int16_t)((int32_t)out->ax_mg + sv);
  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->ay_mg = (int16_t)((int32_t)out->ay_mg + sv);
  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->az_mg = (int16_t)((int32_t)out->az_mg + sv);

  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->gx_dps_x10 = (int16_t)((int32_t)out->gx_dps_x10 + sv);
  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->gy_dps_x10 = (int16_t)((int32_t)out->gy_dps_x10 + sv);
  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->gz_dps_x10 = (int16_t)((int32_t)out->gz_dps_x10 + sv);

  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->vz_cms = (int16_t)((int32_t)out->vz_cms + sv);
  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->vbat_mv = (int16_t)((int32_t)out->vbat_mv + sv);

  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->bmp_stale = clamp_u16((int32_t)out->bmp_stale + sv);
  if (get_svar(payload, len, &off, &sv) != 0) return -1;
  out->imu_stale = clamp_u16((int32_t)out->imu_stale + sv);

  if (get_uvar(payload, len, &off, &uv) != 0) return -1;
  out->flags ^= uv;
  if (get_uvar(payload, len, &off, &uv) != 0) return -1;
  out->flight_state = (uint8_t)uv;
  if (get_uvar(payload, len, &off, &uv) != 0) return -1;
  out->reset_cause = (uint8_t)uv;

  if (off != len) {
    return -1;
  }

  *prev = *out;
  return 0;
}

static void salvage_buffer_to_ram(void) {
  uint32_t off = 0;
  log_rec_t prev = {0};
  log_rec_t rec = {0};
  int have_prev = 0;

  while ((off + 4u) <= g_buf_used) {
    uint8_t tag = g_buf[off++];
    uint8_t len = g_buf[off++];
    if ((off + (uint32_t)len + 2u) > g_buf_used) {
      break;
    }

    uint16_t crc_saved = (uint16_t)g_buf[off + len] | ((uint16_t)g_buf[off + len + 1u] << 8);
    uint16_t crc_calc = crc16_ccitt(&g_buf[off - 2u], 2u + (uint32_t)len);
    if (crc_saved == crc_calc) {
      if (decode_packet_payload(tag, &g_buf[off], len, &rec, &prev, &have_prev) == 0) {
        ram_push(&rec);
      } else {
        g_drop_count++;
      }
    } else {
      g_drop_count++;
    }

    off += (uint32_t)len + 2u;
  }

  g_buf_used = 0;
  g_have_prev = 0;
  g_force_key = 1;
  g_need_key = 1;
  g_since_key = 0;
}

static void enter_fallback(const log_rec_t* current) {
  g_flash_fail_count++;
  g_flash_ok = 0u;
  salvage_buffer_to_ram();
  if (current != 0) {
    ram_push(current);
  }
}

static int log_flush_ram_to_flash(void) {
  uint8_t pkt[LOG_MAX_PACKET];
  uint32_t pkt_len = 0;

  while (g_ram_count > 0u) {
    log_rec_t rec = {0};
    fault_watchdog_kick();
    if (ram_peek_oldest(&rec) != 0) {
      return -1;
    }

    if (encode_record_packet(&rec, pkt, sizeof(pkt), &pkt_len) != 0) {
      return -2;
    }
    if (log_flush_chunk(pkt, pkt_len) != 0) {
      return -3;
    }
    ram_pop_oldest();
  }
  return 0;
}

static void dump_rec_uart(const log_rec_t* r, uint16_t crc_ok) {
  char line[300];
  int n = snprintf(
      line, sizeof(line),
      "%lu,%lu,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,0x%08lX,%u\r\n",
      (unsigned long)r->ms,
      (unsigned long)r->seq,
      (long)r->pressure_pa,
      (long)r->altitude_cm,
      r->temp_centi,
      r->ax_mg, r->ay_mg, r->az_mg,
      r->gx_dps_x10, r->gy_dps_x10, r->gz_dps_x10,
      r->vz_cms,
      r->vbat_mv,
      (unsigned)r->bmp_stale,
      (unsigned)r->imu_stale,
      (unsigned)r->flight_state,
      (unsigned)r->reset_cause,
      (unsigned long)r->flags,
      crc_ok);
  uart_write_len(line, n, sizeof(line));
}

static void dump_ram_records_uart(void) {
  if (g_ram_count == 0u) {
    return;
  }

  uint16_t start = ram_oldest_idx();
  for (uint16_t i = 0; i < g_ram_count; ++i) {
    uint16_t idx = (uint16_t)((start + i) % LOG_RAM_RECS);
    dump_rec_uart(&g_ram[idx], 1u);
  }
}

static int dump_flash_records_uart(void) {
  uint32_t bytes_left = g_used_bytes;
  uint32_t addr = (g_used_bytes == LOG_SIZE) ? g_wr : LOG_BASE;
  uint8_t header[2];
  uint8_t payload[LOG_MAX_PAYLOAD];
  uint8_t crc_bytes[2];
  log_rec_t prev = {0};
  log_rec_t rec = {0};
  int have_prev = 0;

  while (bytes_left >= 4u) {
    fault_watchdog_kick();
    if (flash_read_wrap(addr, header, sizeof(header)) != 0) {
      return -1;
    }
    addr_advance(&addr, sizeof(header));
    bytes_left -= sizeof(header);

    uint8_t tag = header[0];
    uint8_t len = header[1];
    if (len > LOG_MAX_PAYLOAD || bytes_left < ((uint32_t)len + 2u)) {
      break;
    }

    if (flash_read_wrap(addr, payload, len) != 0) {
      return -2;
    }
    addr_advance(&addr, len);
    bytes_left -= len;

    if (flash_read_wrap(addr, crc_bytes, sizeof(crc_bytes)) != 0) {
      return -3;
    }
    addr_advance(&addr, sizeof(crc_bytes));
    bytes_left -= sizeof(crc_bytes);

    uint16_t crc_saved = (uint16_t)crc_bytes[0] | ((uint16_t)crc_bytes[1] << 8);
    uint8_t scratch[2u + LOG_MAX_PAYLOAD];
    scratch[0] = tag;
    scratch[1] = len;
    memcpy(&scratch[2], payload, len);
    uint16_t crc_calc = crc16_ccitt(scratch, (uint32_t)len + 2u);

    if (crc_saved != crc_calc) {
      g_drop_count++;
      continue;
    }

    if (decode_packet_payload(tag, payload, len, &rec, &prev, &have_prev) != 0) {
      g_drop_count++;
      continue;
    }

    dump_rec_uart(&rec, 1u);
  }

  return 0;
}

int log_init(void) {
  g_wr = LOG_BASE;
  g_used_bytes = 0;
  g_seq = 0;
  g_since_key = 0;
  g_records_total = 0;

  g_buf_used = 0;
  g_flash_ok = 0;
  g_force_key = 1;
  g_need_key = 1;
  g_have_prev = 0;
  memset(&g_prev, 0, sizeof(g_prev));

  g_drop_count = 0;
  g_flash_fail_count = 0;
  g_ram_wr = 0;
  g_ram_count = 0;

  if (flash_probe() != 0) {
    return -1;
  }
  if (w25q_erase_4k(LOG_BASE) != 0) {
    return -2;
  }

  g_flash_ok = 1u;
  return 0;
}

void log_append(const log_rec_t* r_in) {
  uint8_t pkt[LOG_MAX_PACKET];
  uint32_t pkt_len = 0;
  int copied = 0;

  if (r_in == 0) {
    return;
  }

  log_rec_t r = *r_in;
  r.seq = g_seq++;
  g_records_total++;

  if ((!g_flash_ok) || (g_ram_count > 0u)) {
    ram_push(&r);
    return;
  }

  if (g_buf_used != 0u && (sizeof(g_buf) - g_buf_used) < LOG_MAX_PACKET) {
    if (flush_buffer_to_flash() != 0) {
      enter_fallback(&r);
      return;
    }
  }

  if (encode_record_packet(&r, pkt, sizeof(pkt), &pkt_len) != 0) {
    enter_fallback(&r);
    return;
  }

  if (append_packet(pkt, pkt_len, &copied) != 0) {
    (void)copied;
    enter_fallback(&r);
  }
}

void log_flush(void) {
  if (!g_flash_ok) {
    if (flash_probe() == 0) {
      g_flash_ok = 1u;
      g_force_key = 1;
      g_need_key = 1;
      g_have_prev = 0;
      g_since_key = 0;
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
      g_force_key = 1;
      g_need_key = 1;
      g_have_prev = 0;
      g_since_key = 0;
      return;
    }
  }

  if (flush_buffer_to_flash() != 0) {
    g_flash_fail_count++;
    g_flash_ok = 0u;
    salvage_buffer_to_ram();
  }
}

void log_dump_uart(void) {
  log_flush();

  uart2_puts("DUMP_BEGIN\r\n");
  if (g_flash_ok && (g_ram_count == 0u)) {
    uart2_puts("#mode,flash\r\n");
  } else {
    uart2_puts("#mode,ram_fallback\r\n");
  }

  char meta[220];
  int m = snprintf(meta, sizeof(meta),
                   "#records,%lu\r\n#bytes_used,%lu\r\n#dropped,%lu\r\n#flash_fail,%lu\r\n#ram_pending,%u\r\n",
                   (unsigned long)g_records_total,
                   (unsigned long)g_used_bytes,
                   (unsigned long)g_drop_count,
                   (unsigned long)g_flash_fail_count,
                   (unsigned)g_ram_count);
  uart_write_len(meta, m, sizeof(meta));

  uart2_puts("ms,seq,pressure_pa,altitude_cm,temp_centi,ax_mg,ay_mg,az_mg,gx_dps_x10,gy_dps_x10,gz_dps_x10,vz_cms,vbat_mv,bmp_stale,imu_stale,flight_state,reset_cause,flags,crc_ok\r\n");

  if (g_flash_ok && (g_used_bytes > 0u)) {
    if (dump_flash_records_uart() != 0) {
      g_flash_ok = 0u;
      g_flash_fail_count++;
    }
  }

  dump_ram_records_uart();
  uart2_puts("DUMP_END\r\n");
}

int log_is_flash_enabled(void) {
  return g_flash_ok ? 1 : 0;
}

uint32_t log_dropped_records(void) {
  return g_drop_count;
}
