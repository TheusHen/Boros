#pragma once
#include <stdint.h>

#define PERIPH_BASE     0x40000000UL
#define AHB2PERIPH_BASE (PERIPH_BASE + 0x08000000UL)
#define APB1PERIPH_BASE (PERIPH_BASE + 0x00000000UL)
#define APB2PERIPH_BASE (PERIPH_BASE + 0x00010000UL)

#define RCC_BASE   (0x40021000UL)
#define GPIOA_BASE (AHB2PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE (AHB2PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE (AHB2PERIPH_BASE + 0x0800UL)
#define SPI1_BASE  (APB2PERIPH_BASE + 0x3000UL)
#define USART2_BASE (APB1PERIPH_BASE + 0x4400UL)
#define FLASH_BASE (0x40022000UL)
#define SYSTICK_BASE (0xE000E010UL)
#define NVIC_ISER0  (*(volatile uint32_t*)0xE000E100UL)

typedef struct {
  volatile uint32_t CR;
  volatile uint32_t ICSCR;
  volatile uint32_t CFGR;
  volatile uint32_t PLLSYSCFGR;
  volatile uint32_t RESERVED0;
  volatile uint32_t CIER;
  volatile uint32_t CIFR;
  volatile uint32_t CICR;
  volatile uint32_t IOPRSTR;
  volatile uint32_t AHBRSTR;
  volatile uint32_t APBRSTR1;
  volatile uint32_t APBRSTR2;
  volatile uint32_t IOPENR;
  volatile uint32_t AHBENR;
  volatile uint32_t APBENR1;
  volatile uint32_t APBENR2;
  volatile uint32_t IOPSMENR;
  volatile uint32_t AHBSMENR;
  volatile uint32_t APBSMENR1;
  volatile uint32_t APBSMENR2;
  volatile uint32_t CCIPR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
} RCC_TypeDef;

typedef struct {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFRL;
  volatile uint32_t AFRH;
  volatile uint32_t BRR;
} GPIO_TypeDef;

typedef struct {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
} SPI_TypeDef;

typedef struct {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t BRR;
  volatile uint32_t GTPR;
  volatile uint32_t RTOR;
  volatile uint32_t RQR;
  volatile uint32_t ISR;
  volatile uint32_t ICR;
  volatile uint32_t RDR;
  volatile uint32_t TDR;
  volatile uint32_t PRESC;
} USART_TypeDef;

typedef struct {
  volatile uint32_t ACR;
} FLASH_TypeDef;

typedef struct {
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile uint32_t CALIB;
} SysTick_TypeDef;

#define RCC   ((RCC_TypeDef*)RCC_BASE)
#define GPIOA ((GPIO_TypeDef*)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef*)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef*)GPIOC_BASE)
#define SPI1  ((SPI_TypeDef*)SPI1_BASE)
#define USART2 ((USART_TypeDef*)USART2_BASE)
#define FLASH ((FLASH_TypeDef*)FLASH_BASE)
#define SysTick ((SysTick_TypeDef*)SYSTICK_BASE)