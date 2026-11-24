/*
 * semihost.h - ARM semihosting: the program asks the debugger/emulator to do
 * I/O for it with a BKPT 0xAB instruction. QEMU handles it when started with
 * -semihosting-config enable=on. On a real board without a debugger attached
 * these calls hard-fault, so the firmware only uses them for emulation.
 */
#ifndef SEMIHOST_H
#define SEMIHOST_H

#include <stdint.h>

static inline int sh_call(int op, const void *arg)
{
    register int r0 __asm__("r0") = op;
    register const void *r1 __asm__("r1") = arg;
    __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
    return r0;
}

static inline void sh_write0(const char *s) { sh_call(0x04, s); }

static inline void sh_exit(int code)
{
    /* SYS_EXIT_EXTENDED (0x20) with ADP_Stopped_ApplicationExit and a code */
    static uint32_t block[2];
    block[0] = 0x20026u;
    block[1] = (uint32_t)code;
    sh_call(0x20, block);
}

/* key=value line writer without printf (printf would pollute the stack measurement) */
static inline void sh_kv(const char *key, uint32_t v)
{
    char buf[48];
    int n = 0;
    while (*key && n < 30) buf[n++] = *key++;
    buf[n++] = '=';
    char tmp[12];
    int k = 0;
    do { tmp[k++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (k) buf[n++] = tmp[--k];
    buf[n++] = '\n';
    buf[n] = 0;
    sh_write0(buf);
}

#endif
