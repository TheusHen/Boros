#pragma once

#include <stdint.h>

enum {
  RESET_CAUSE_PIN       = (1u << 0),
  RESET_CAUSE_SOFT      = (1u << 1),
  RESET_CAUSE_WDT       = (1u << 2),
  RESET_CAUSE_BOR       = (1u << 3),
  RESET_CAUSE_POR       = (1u << 4),
  RESET_CAUSE_LPWR      = (1u << 5),
  RESET_CAUSE_HARDFAULT = (1u << 6),
};

typedef struct {
  uint32_t magic;
  uint32_t reason;
  uint32_t reset_csr;
  uint32_t stacked_r0;
  uint32_t stacked_r1;
  uint32_t stacked_r2;
  uint32_t stacked_r3;
  uint32_t stacked_r12;
  uint32_t stacked_lr;
  uint32_t stacked_pc;
  uint32_t stacked_xpsr;
  uint32_t exc_lr;
  uint32_t crc;
} fault_snapshot_t;

void fault_boot_init(void);
uint32_t fault_get_reset_cause(void);
int fault_had_hardfault(void);
const volatile fault_snapshot_t* fault_snapshot(void);

void fault_watchdog_init(uint32_t timeout_ms);
void fault_watchdog_kick(void);

void fault_hardfault(uint32_t* stacked, uint32_t exc_lr);
