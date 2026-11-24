/*
 * pil_main.c - processor-in-the-loop learning: the learner lives on the chip.
 *
 *   laptop (python/tinyrl/pil.py)                         Cortex-M4 (this file)
 *   task (MuJoCo or corridor), logs  --- UART frames --->   tr_init / tr_act / tr_observe
 *                                   <--- action, cycles -- in a static arena in SRAM
 *
 * Frame (both directions):  0xA5 | type u8 | len u16 | payload[len] | sum u8 (payload bytes mod 256)
 * All numbers little-endian; observations, rewards, epsilon are Q16.16.
 *
 *   0x00 HELLO                            -> 0x80 version, sizeof(tr_cfg_t), TR_MAX_OBS, arena capacity, CPU Hz
 *   0x01 INIT  cfg bytes + u32 seed       -> 0x81 i32 rc, u32 arena bytes used, u32 sizeof(tr_agent_t)
 *   0x02 ACT   obs[d] + eps               -> 0x82 u8 action, u32 cycles
 *   0x03 GREEDY obs[d]                    -> 0x83 u8 action, u32 cycles
 *   0x04 OBSERVE s[d] u8 a r s2[d] u8 done -> 0x84 i32 updates, u32 cycles
 *   0x05 EPISODE                          -> 0x85 (traces cleared)
 *   0x06 STATS                            -> 0x86 u32 peak stack bytes, u32 max act cycles, u32 max observe cycles
 *   anything malformed                    -> 0xFF u8 error code
 *
 * Because the learner code is the same C as the host build, a PIL run must reproduce
 * the host run bit for bit (scripts/pil_check.py checks this in QEMU). On the Nucleo,
 * the new information is the cycle count of every call.
 */
#include <stdint.h>
#include <string.h>
#include "tinyrl.h"
#include "board.h"

#define PIL_VERSION 1u
#ifndef PIL_ARENA
#define PIL_ARENA (88u * 1024u)
#endif
#define MAX_PAYLOAD (sizeof(tr_cfg_t) + 64u)
#define PAINT 0xA5A5A5A5u

extern uint32_t _ebss;

static tr_agent_t ag;
static tr_cfg_t cfg;
static uint8_t arena[PIL_ARENA] __attribute__((aligned(4)));
static uint8_t buf[MAX_PAYLOAD + 8u];
static uint8_t out[32];
static int ready;
static uint32_t max_act, max_obs, sp_top;

static void send(uint8_t type, const uint8_t *p, uint16_t n)
{
    uint8_t sum = 0;
    uart_put(0xA5); uart_put(type); uart_put((uint8_t)n); uart_put((uint8_t)(n >> 8));
    for (uint16_t i = 0; i < n; i++) { uart_put(p[i]); sum = (uint8_t)(sum + p[i]); }
    uart_put(sum);
}

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static int32_t get32(const uint8_t *p) { return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24); }
static void err(uint8_t code) { send(0xFF, &code, 1); }

/* obs arrives as bytes; copy into aligned Q16.16 arrays */
static void load_obs(const uint8_t *p, q16_t *o) { for (int i = 0; i < cfg.obs_dim; i++) o[i] = get32(p + 4 * i); }

static uint32_t stack_peak(void)
{
    uint32_t *low = &_ebss;
    while (low < (uint32_t *)sp_top && *low == PAINT) low++;
    return sp_top - (uint32_t)low;
}

static void handle(uint8_t type, const uint8_t *p, uint16_t n)
{
    static q16_t s[TR_MAX_OBS], s2[TR_MAX_OBS];
    const uint32_t d4 = 4u * (uint32_t)cfg.obs_dim;
    uint32_t t0, dt;
    switch (type) {
    case 0x00:
        put32(out, PIL_VERSION); put32(out + 4, sizeof(tr_cfg_t)); put32(out + 8, TR_MAX_OBS);
        put32(out + 12, PIL_ARENA); put32(out + 16, CPU_HZ);
        send(0x80, out, 20);
        return;
    case 0x01: {
        if (n != sizeof(tr_cfg_t) + 4u) { err(1); return; }
        memcpy(&cfg, p, sizeof(tr_cfg_t));
        int rc = tr_init(&ag, &cfg, arena, sizeof(arena), (uint32_t)get32(p + sizeof(tr_cfg_t)));
        ready = rc == TR_OK;
        max_act = max_obs = 0;
        put32(out, (uint32_t)rc); put32(out + 4, ag.arena_used); put32(out + 8, sizeof(tr_agent_t));
        send(0x81, out, 12);
        return;
    }
    }
    if (!ready) { err(2); return; }
    switch (type) {
    case 0x02:
    case 0x03: {
        if (n != d4 + (type == 0x02 ? 4u : 0u)) { err(3); return; }
        load_obs(p, s);
        t0 = cycles_now();
        int a = type == 0x02 ? tr_act(&ag, s, get32(p + d4)) : tr_greedy(&ag, s);
        dt = cycles_now() - t0;
        if (dt > max_act) max_act = dt;
        out[0] = (uint8_t)a; put32(out + 1, dt);
        send((uint8_t)(type | 0x80u), out, 5);
        return;
    }
    case 0x04: {
        if (n != 2u * d4 + 6u) { err(3); return; }
        load_obs(p, s);
        int a = p[d4];
        q16_t r = get32(p + d4 + 1);
        load_obs(p + d4 + 5, s2);
        int done = p[2u * d4 + 5u];
        t0 = cycles_now();
        int k = tr_observe(&ag, s, a, r, s2, done);
        dt = cycles_now() - t0;
        if (dt > max_obs) max_obs = dt;
        put32(out, (uint32_t)k); put32(out + 4, dt);
        send(0x84, out, 8);
        return;
    }
    case 0x05:
        tr_episode_start(&ag);
        send(0x85, out, 0);
        return;
    case 0x06:
        put32(out, stack_peak()); put32(out + 4, max_act); put32(out + 8, max_obs);
        send(0x86, out, 12);
        return;
    default:
        err(4);
    }
}

int main(void)
{
    uint32_t sp;
    __asm__ volatile("mov %0, sp" : "=r"(sp));
    for (uint32_t *q = &_ebss; q < (uint32_t *)(sp - 256u); q++) *q = PAINT;
    sp_top = sp;
    board_clock_init();
    board_uart_init();
    cycles_init();
    for (;;) {
        while (uart_get() != 0xA5) {}                    /* resynchronise on the start byte */
        uint8_t type = uart_get();
        uint16_t n = uart_get();
        n |= (uint16_t)uart_get() << 8;
        if (n > MAX_PAYLOAD) { err(5); continue; }
        uint8_t sum = 0;
        for (uint16_t i = 0; i < n; i++) { buf[i] = uart_get(); sum = (uint8_t)(sum + buf[i]); }
        if (uart_get() != sum) { err(6); continue; }
        handle(type, buf, n);
    }
}
