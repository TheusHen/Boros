#include "fault.h"

#include "stm32g071.h"

#define SNAPSHOT_MAGIC 0x48534654u /* "HSFT" */
#define SNAPSHOT_REASON_HARDFAULT 1u

#define RCC_CSR_RMVF      (1u << 23)
#define RCC_CSR_BORRSTF   (1u << 25)
#define RCC_CSR_PINRSTF   (1u << 26)
#define RCC_CSR_PORRSTF   (1u << 27)
#define RCC_CSR_SFTRSTF   (1u << 28)
#define RCC_CSR_IWDGRSTF  (1u << 29)
#define RCC_CSR_WWDGRSTF  (1u << 30)
#define RCC_CSR_LPWRRSTF  (1u << 31)

#define IWDG_KR_UNLOCK 0x5555u
#define IWDG_KR_START  0xCCCCu
#define IWDG_KR_RELOAD 0xAAAAu

#define SCB_AIRCR (*(volatile uint32_t*)0xE000ED0Cu)
#define SCB_AIRCR_VECTKEY (0x5FAu << 16)
#define SCB_AIRCR_SYSRESETREQ (1u << 2)

__attribute__((section(".noinit"))) static fault_snapshot_t g_snapshot;

static uint32_t g_reset_cause = 0;
static uint8_t g_had_hardfault = 0;
static uint8_t g_wdt_enabled = 0;

static uint32_t checksum_snapshot(const volatile fault_snapshot_t* s) {
  const volatile uint32_t* p = (const volatile uint32_t*)s;
  uint32_t crc = 0xA5A5A5A5u;
  for (uint32_t i = 0; i < (sizeof(fault_snapshot_t) / sizeof(uint32_t)) - 1u; ++i) {
    crc ^= p[i] + (i * 0x9E3779B9u);
    crc = (crc << 5) | (crc >> 27);
  }
  return crc;
}

static void system_reset_now(void) {
  SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
  while (1) {
  }
}

void fault_boot_init(void) {
  uint32_t csr = RCC->CSR;
  g_reset_cause = 0;

  if (csr & RCC_CSR_PINRSTF)  g_reset_cause |= RESET_CAUSE_PIN;
  if (csr & RCC_CSR_SFTRSTF)  g_reset_cause |= RESET_CAUSE_SOFT;
  if (csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) g_reset_cause |= RESET_CAUSE_WDT;
  if (csr & RCC_CSR_BORRSTF)  g_reset_cause |= RESET_CAUSE_BOR;
  if (csr & RCC_CSR_PORRSTF)  g_reset_cause |= RESET_CAUSE_POR;
  if (csr & RCC_CSR_LPWRRSTF) g_reset_cause |= RESET_CAUSE_LPWR;

  g_had_hardfault = 0;
  if (g_snapshot.magic == SNAPSHOT_MAGIC && g_snapshot.reason == SNAPSHOT_REASON_HARDFAULT) {
    if (g_snapshot.crc == checksum_snapshot(&g_snapshot)) {
      g_had_hardfault = 1u;
      g_reset_cause |= RESET_CAUSE_HARDFAULT;
    }
  }

  g_snapshot.magic = 0;
  g_snapshot.reason = 0;
  g_snapshot.crc = 0;

  RCC->CSR |= RCC_CSR_RMVF;
}

uint32_t fault_get_reset_cause(void) {
  return g_reset_cause;
}

int fault_had_hardfault(void) {
  return g_had_hardfault ? 1 : 0;
}

const volatile fault_snapshot_t* fault_snapshot(void) {
  return &g_snapshot;
}

void fault_watchdog_init(uint32_t timeout_ms) {
  uint32_t reload = timeout_ms / 8u;
  if (reload == 0u) {
    reload = 1u;
  }
  if (reload > 4095u) {
    reload = 4095u;
  }

  IWDG->KR = IWDG_KR_UNLOCK;
  IWDG->PR = 6u;           /* /256 */
  IWDG->RLR = reload;
  while ((IWDG->SR & 0x7u) != 0u) {
  }

  IWDG->KR = IWDG_KR_START;
  IWDG->KR = IWDG_KR_RELOAD;
  g_wdt_enabled = 1u;
}

void fault_watchdog_kick(void) {
  if (g_wdt_enabled) {
    IWDG->KR = IWDG_KR_RELOAD;
  }
}

void fault_hardfault(uint32_t* stacked, uint32_t exc_lr) {
  g_snapshot.magic = SNAPSHOT_MAGIC;
  g_snapshot.reason = SNAPSHOT_REASON_HARDFAULT;
  g_snapshot.reset_csr = RCC->CSR;
  g_snapshot.exc_lr = exc_lr;

  if (stacked != 0) {
    g_snapshot.stacked_r0 = stacked[0];
    g_snapshot.stacked_r1 = stacked[1];
    g_snapshot.stacked_r2 = stacked[2];
    g_snapshot.stacked_r3 = stacked[3];
    g_snapshot.stacked_r12 = stacked[4];
    g_snapshot.stacked_lr = stacked[5];
    g_snapshot.stacked_pc = stacked[6];
    g_snapshot.stacked_xpsr = stacked[7];
  } else {
    g_snapshot.stacked_r0 = 0;
    g_snapshot.stacked_r1 = 0;
    g_snapshot.stacked_r2 = 0;
    g_snapshot.stacked_r3 = 0;
    g_snapshot.stacked_r12 = 0;
    g_snapshot.stacked_lr = 0;
    g_snapshot.stacked_pc = 0;
    g_snapshot.stacked_xpsr = 0;
  }

  g_snapshot.crc = checksum_snapshot(&g_snapshot);
  system_reset_now();
}
