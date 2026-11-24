/*
 * board.h - the only board-specific code: UART, cycle counter, clock.
 *
 *   BOARD_QEMU   (default)  QEMU netduinoplus2: USART1 at 0x40011000, attached to QEMU's
 *                           first -serial (here a TCP socket). No cycle counter.
 *   BOARD_NUCLEO            STM32F446RE Nucleo-64: USART2 (PA2/PA3) = the ST-LINK virtual COM
 *                           port, DWT->CYCCNT cycle counter, HSI 16 MHz by default
 *                           (-DCLOCK_180MHZ for PLL + over-drive at 180 MHz).
 *
 * BOARD_NUCLEO is written from the STM32F446 reference manual (RM0390) register map but has
 * NOT been run on hardware yet. Test it on the board before trusting any number from it.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#define REG(addr) (*(volatile uint32_t *)(addr))

/* ---- STM32F4 USART (same register layout on F405 and F446) ---- */
#define USART_SR   0x00u
#define USART_DR   0x04u
#define USART_BRR  0x08u
#define USART_CR1  0x0Cu
#define SR_RXNE    (1u << 5)
#define SR_TXE     (1u << 7)
#define CR1_UE     (1u << 13)
#define CR1_TE     (1u << 3)
#define CR1_RE     (1u << 2)

#if defined(BOARD_NUCLEO)
#define UART_BASE  0x40004400u          /* USART2 */
#ifdef CLOCK_180MHZ
#define CPU_HZ     180000000u
#define APB1_HZ    45000000u
#else
#define CPU_HZ     16000000u
#define APB1_HZ    16000000u
#endif
#else
#define UART_BASE  0x40011000u          /* USART1 in QEMU's stm32f405 model */
#define CPU_HZ     0u                   /* unknown: QEMU is not cycle-accurate */
#endif

#define RCC        0x40023800u
#define PWR        0x40007000u
#define FLASH_ACR  0x40023C00u
#define GPIOA      0x40020000u

static inline void board_clock_init(void)
{
#if defined(BOARD_NUCLEO) && defined(CLOCK_180MHZ)
    REG(RCC + 0x40) |= (1u << 28);                         /* APB1ENR: PWR clock */
    REG(PWR + 0x00) |= (3u << 14);                         /* VOS = scale 1 */
    /* PLL from HSI 16 MHz: M = 8 (2 MHz), N = 180 (VCO 360 MHz), P = 2 -> 180 MHz, Q = 8 */
    REG(RCC + 0x04) = 8u | (180u << 6) | (0u << 16) | (0u << 22) | (8u << 24);
    REG(RCC + 0x00) |= (1u << 24);                         /* PLLON */
    while (!(REG(RCC + 0x00) & (1u << 25))) {}             /* PLLRDY */
    REG(PWR + 0x00) |= (1u << 16);                         /* ODEN */
    while (!(REG(PWR + 0x04) & (1u << 16))) {}             /* ODRDY */
    REG(PWR + 0x00) |= (1u << 17);                         /* ODSWEN */
    while (!(REG(PWR + 0x04) & (1u << 17))) {}             /* ODSWRDY */
    REG(FLASH_ACR) = 5u | (1u << 8) | (1u << 9) | (1u << 10);   /* 5 wait states, prefetch, I/D cache */
    /* AHB /1, APB1 /4 (45 MHz), APB2 /2 (90 MHz), system clock = PLL */
    REG(RCC + 0x08) = (REG(RCC + 0x08) & ~0xFCF3u) | (5u << 10) | (4u << 13) | 2u;
    while (((REG(RCC + 0x08) >> 2) & 3u) != 2u) {}        /* SWS = PLL */
#endif
}

static inline void board_uart_init(void)
{
#if defined(BOARD_NUCLEO)
    REG(RCC + 0x30) |= 1u;                                 /* AHB1ENR: GPIOA */
    REG(RCC + 0x40) |= (1u << 17);                         /* APB1ENR: USART2 */
    REG(GPIOA + 0x00) = (REG(GPIOA + 0x00) & ~(0xFu << 4)) | (0xAu << 4);   /* PA2, PA3 alternate */
    REG(GPIOA + 0x20) = (REG(GPIOA + 0x20) & ~(0xFFu << 8)) | (0x77u << 8);  /* AF7 = USART2 */
    REG(UART_BASE + USART_BRR) = (APB1_HZ + 115200u / 2u) / 115200u;        /* 115200 baud, 16x oversampling */
#endif
    REG(UART_BASE + USART_CR1) = CR1_UE | CR1_TE | CR1_RE;
}

static inline void uart_put(uint8_t b)
{
    while (!(REG(UART_BASE + USART_SR) & SR_TXE)) {}
    REG(UART_BASE + USART_DR) = b;
}

static inline uint8_t uart_get(void)
{
    while (!(REG(UART_BASE + USART_SR) & SR_RXNE)) {}
    return (uint8_t)REG(UART_BASE + USART_DR);
}

/* ---- cycle counter: DWT->CYCCNT on the real chip, absent in QEMU ---- */
static inline void cycles_init(void)
{
#if defined(BOARD_NUCLEO)
    REG(0xE000EDFCu) |= (1u << 24);                        /* DEMCR.TRCENA */
    REG(0xE0001004u) = 0;                                  /* DWT_CYCCNT */
    REG(0xE0001000u) |= 1u;                                /* DWT_CTRL.CYCCNTENA */
#endif
}

static inline uint32_t cycles_now(void)
{
#if defined(BOARD_NUCLEO)
    return REG(0xE0001004u);
#else
    return 0u;
#endif
}

#endif
