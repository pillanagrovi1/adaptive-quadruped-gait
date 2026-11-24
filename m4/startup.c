/*
 * startup.c - minimal Cortex-M4 reset path: copy .data, zero .bss, call main,
 * then tell QEMU to exit via semihosting.
 */
#include <stdint.h>
#include "semihost.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);

void Reset_Handler(void)
{
#if defined(__ARM_FP)
    /* hard-float builds (m4/kernels): enable the FPU, CP10 + CP11 full access */
    *(volatile uint32_t *)0xE000ED88u |= (0xFu << 20);
    __asm__ volatile("dsb\n isb" ::: "memory");
#endif
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
    int rc = main();
    sh_exit(rc);
    for (;;) {}
}

void Default_Handler(void)
{
    sh_write0("FAULT\n");
    sh_exit(99);
    for (;;) {}
}

__attribute__((section(".isr_vector"), used))
const void *const vector_table[16] = {
    (void *)&_estack,          /* initial stack pointer */
    (void *)Reset_Handler,
    (void *)Default_Handler,   /* NMI */
    (void *)Default_Handler,   /* HardFault */
    (void *)Default_Handler,   /* MemManage */
    (void *)Default_Handler,   /* BusFault */
    (void *)Default_Handler,   /* UsageFault */
    0, 0, 0, 0,
    (void *)Default_Handler,   /* SVC */
    (void *)Default_Handler,   /* DebugMon */
    0,
    (void *)Default_Handler,   /* PendSV */
    (void *)Default_Handler,   /* SysTick */
};
