#include "bmp390.h"

#include "gpio.h"
#include "spi.h"

#define BMP390_REG_CHIP_ID  0x00
#define BMP390_REG_PDATA    0x04
#define BMP390_REG_PWR_CTRL 0x1B
#define BMP390_REG_OSR      0x1C
#define BMP390_REG_ODR      0x1D
#define BMP390_REG_CONFIG   0x1F
#define BMP390_REG_CALIB    0x31
#define BMP390_REG_CMD      0x7E

#define BMP390_CHIP_ID     0x60
#define BMP390_SOFT_RESET  0xB6

typedef struct {
  uint16_t par_t1;
  uint16_t par_t2;
  int8_t par_t3;
  int16_t par_p1;
  int16_t par_p2;
  int8_t par_p3;
  int8_t par_p4;
  uint16_t par_p5;
  uint16_t par_p6;
  int8_t par_p7;
  int8_t par_p8;
  int16_t par_p9;
  int8_t par_p10;
  int8_t par_p11;
  int64_t t_lin;
} bmp390_calib_t;

static bmp390_calib_t g_cal = {0};

static int32_t sat_i32(int64_t v) {
  if (v > 2147483647LL) return 2147483647;
  if (v < -2147483648LL) return -2147483648;
  return (int32_t)v;
}

/* BMP390 SPI reads require one dummy byte after register address. */
static void bmp_rn(uint8_t reg, uint8_t* out, uint32_t n) {
  uint8_t addr = (uint8_t)(reg | 0x80);
  uint8_t dummy = 0;
  cs_baro(0);
  spi1_txrx(&addr, 0, 1);
  spi1_txrx(0, &dummy, 1);
  spi1_txrx(0, out, n);
  cs_baro(1);
}

static uint8_t bmp_r(uint8_t reg) {
  uint8_t v = 0;
  bmp_rn(reg, &v, 1);
  return v;
}

static void bmp_w(uint8_t reg, uint8_t val) {
  uint8_t tx[2] = {(uint8_t)(reg & 0x7F), val};
  cs_baro(0);
  spi1_txrx(tx, 0, 2);
  cs_baro(1);
}

static int32_t bmp390_comp_temp_centi(int32_t uncomp_temp) {
  int64_t pd1 = (int64_t)(uncomp_temp - ((int64_t)256 * g_cal.par_t1));
  int64_t pd2 = (int64_t)g_cal.par_t2 * pd1;
  int64_t pd3 = pd1 * pd1;
  int64_t pd4 = pd3 * g_cal.par_t3;
  int64_t pd5 = (pd2 * 262144) + pd4;
  int64_t pd6 = pd5 / 4294967296LL;
  g_cal.t_lin = pd6;
  return sat_i32((pd6 * 25) / 16384);
}

static int32_t bmp390_comp_press_pa(int32_t uncomp_press) {
  int64_t pd1 = (int64_t)(g_cal.t_lin * g_cal.t_lin);
  int64_t pd2 = pd1 / 64;
  int64_t pd3 = (pd2 * g_cal.t_lin) / 256;
  int64_t pd4 = (g_cal.par_p8 * pd3) / 32;
  int64_t pd5 = (g_cal.par_p7 * pd1) * 16;
  int64_t pd6 = (g_cal.par_p6 * g_cal.t_lin) * 4194304;
  int64_t offset = ((int64_t)g_cal.par_p5 * 140737488355328LL) + pd4 + pd5 + pd6;

  pd2 = (g_cal.par_p4 * pd3) / 32;
  pd4 = (g_cal.par_p3 * pd1) * 4;
  pd5 = ((int64_t)(g_cal.par_p2 - 16384) * g_cal.t_lin) * 2097152;
  int64_t sensitivity =
      ((int64_t)(g_cal.par_p1 - 16384) * 70368744177664LL) + pd2 + pd4 + pd5;

  pd1 = (sensitivity / 16777216LL) * uncomp_press;
  pd2 = (int64_t)g_cal.par_p10 * g_cal.t_lin;
  pd3 = pd2 + ((int64_t)65536 * g_cal.par_p9);
  pd4 = (pd3 * uncomp_press) / 8192;

  pd5 = (uncomp_press * (pd4 / 10)) / 512;
  pd5 *= 10;

  pd6 = (int64_t)uncomp_press * uncomp_press;
  pd2 = ((int64_t)g_cal.par_p11 * pd6) / 65536;
  pd3 = (pd2 * uncomp_press) / 128;
  pd4 = (offset / 4) + pd1 + pd5 + pd3;

  if (pd4 < 0) return 0;

  /* Result is pressure in Pa*100. */
  uint64_t comp_press_x100 = ((uint64_t)pd4 * 25u) / 1099511627776ULL;
  return sat_i32((int64_t)(comp_press_x100 / 100u));
}

int bmp390_init(void) {
  uint8_t id = bmp_r(BMP390_REG_CHIP_ID);
  if (id != BMP390_CHIP_ID) return -1;

  bmp_w(BMP390_REG_CMD, BMP390_SOFT_RESET);
  for (volatile int i = 0; i < 100000; i++) { }

  id = bmp_r(BMP390_REG_CHIP_ID);
  if (id != BMP390_CHIP_ID) return -2;

  uint8_t calib[21] = {0};
  bmp_rn(BMP390_REG_CALIB, calib, sizeof(calib));
  g_cal.par_t1 = (uint16_t)((calib[1] << 8) | calib[0]);
  g_cal.par_t2 = (uint16_t)((calib[3] << 8) | calib[2]);
  g_cal.par_t3 = (int8_t)calib[4];
  g_cal.par_p1 = (int16_t)((calib[6] << 8) | calib[5]);
  g_cal.par_p2 = (int16_t)((calib[8] << 8) | calib[7]);
  g_cal.par_p3 = (int8_t)calib[9];
  g_cal.par_p4 = (int8_t)calib[10];
  g_cal.par_p5 = (uint16_t)((calib[12] << 8) | calib[11]);
  g_cal.par_p6 = (uint16_t)((calib[14] << 8) | calib[13]);
  g_cal.par_p7 = (int8_t)calib[15];
  g_cal.par_p8 = (int8_t)calib[16];
  g_cal.par_p9 = (int16_t)((calib[18] << 8) | calib[17]);
  g_cal.par_p10 = (int8_t)calib[19];
  g_cal.par_p11 = (int8_t)calib[20];
  g_cal.t_lin = 0;

  bmp_w(BMP390_REG_OSR, 0x0Bu);      /* osr_t=x2, osr_p=x8 */
  bmp_w(BMP390_REG_ODR, 0x03u);      /* moderate ODR */
  bmp_w(BMP390_REG_CONFIG, 0x04u);   /* IIR filter */
  bmp_w(BMP390_REG_PWR_CTRL, 0x33u); /* press+temp enabled, normal mode */
  for (volatile int i = 0; i < 30000; i++) { }

  return 0;
}

int bmp390_read_raw(int32_t* press_raw, int32_t* temp_raw) {
  uint8_t rx[6] = {0};
  bmp_rn(BMP390_REG_PDATA, rx, sizeof(rx));

  *press_raw = (int32_t)((uint32_t)rx[0] | ((uint32_t)rx[1] << 8) | ((uint32_t)rx[2] << 16));
  *temp_raw = (int32_t)((uint32_t)rx[3] | ((uint32_t)rx[4] << 8) | ((uint32_t)rx[5] << 16));
  return 0;
}

int bmp390_read_sample(bmp390_sample_t* s) {
  if (!s) return -1;

  if (bmp390_read_raw(&s->press_raw, &s->temp_raw) != 0) {
    return -2;
  }

  int32_t temp_centi = bmp390_comp_temp_centi(s->temp_raw);
  int32_t press_pa = bmp390_comp_press_pa(s->press_raw);

  if (temp_centi > 32767) temp_centi = 32767;
  if (temp_centi < -32768) temp_centi = -32768;

  s->temp_centi = (int16_t)temp_centi;
  s->press_pa = press_pa;
  return 0;
}
