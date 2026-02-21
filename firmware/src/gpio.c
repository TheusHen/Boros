// gpio.c
#include "stm32g071.h"
#include "gpio.h"

// SPI1: PA5 SCK, PA6 MISO, PA7 MOSI
// CS: PA4 FLASH_CS, PB0 IMU_CS, PB1 BARO_CS
// BUZZER: PB9
// UART2: PA2 TX, PA3 RX (AF1)
// USB: PA11/PA12
// Status inputs: PA8 USB_VBUS_SENSE, PC15 CHG_STAT, PB6 IMU_INT1, PB7 IMU_INT2

static inline void gpio_mode(GPIO_TypeDef* p, int pin, int mode) {
  p->MODER = (p->MODER & ~(3u << (pin*2))) | ((uint32_t)mode << (pin*2));
}
static inline void gpio_af(GPIO_TypeDef* p, int pin, int af) {
  if (pin < 8) {
    p->AFRL = (p->AFRL & ~(0xFu << (pin*4))) | ((uint32_t)af << (pin*4));
  } else {
    int p8 = pin - 8;
    p->AFRH = (p->AFRH & ~(0xFu << (p8*4))) | ((uint32_t)af << (p8*4));
  }
}

void gpio_init_all(void) {
  gpio_mode(GPIOA, 4, 1);
  gpio_mode(GPIOB, 0, 1);
  gpio_mode(GPIOB, 1, 1);
  gpio_mode(GPIOB, 9, 1);

  cs_flash(1);
  cs_imu(1);
  cs_baro(1);

  gpio_mode(GPIOA, 5, 2);
  gpio_mode(GPIOA, 6, 2);
  gpio_mode(GPIOA, 7, 2);
  gpio_af(GPIOA, 5, 0);
  gpio_af(GPIOA, 6, 0);
  gpio_af(GPIOA, 7, 0);

  gpio_mode(GPIOA, 2, 2);
  gpio_mode(GPIOA, 3, 2);
  gpio_af(GPIOA, 2, 1);
  gpio_af(GPIOA, 3, 1);

  gpio_mode(GPIOA, 8, 0);
  gpio_mode(GPIOC, 15, 0);
  gpio_mode(GPIOB, 6, 0);
  gpio_mode(GPIOB, 7, 0);
}

void cs_flash(int level) { if (level) GPIOA->BSRR = (1u<<4); else GPIOA->BSRR = (1u<<(4+16)); }
void cs_imu(int level)   { if (level) GPIOB->BSRR = (1u<<0); else GPIOB->BSRR = (1u<<(0+16)); }
void cs_baro(int level)  { if (level) GPIOB->BSRR = (1u<<1); else GPIOB->BSRR = (1u<<(1+16)); }

void buzzer_set(int on)  { if (on) GPIOB->BSRR = (1u<<9); else GPIOB->BSRR = (1u<<(9+16)); }

int usb_vbus_sense_read(void) { return (GPIOA->IDR >> 8) & 1u; }
int chg_stat_read(void)       { return (GPIOC->IDR >> 15) & 1u; }
int imu_int1_read(void)       { return (GPIOB->IDR >> 6) & 1u; }
int imu_int2_read(void)       { return (GPIOB->IDR >> 7) & 1u; }
