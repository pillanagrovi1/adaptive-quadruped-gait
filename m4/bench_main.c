/*
 * bench_main.c - measure one learner configuration on a Cortex-M4 (in QEMU).
 *
 * What it reports (semihosting, key=value lines):
 *   agent_bytes     sizeof(tr_agent_t) on THIS target (32-bit pointers)
 *   arena_bytes     bytes of the static arena the learner uses (tr_mem_bytes)
 *   stack_learner   peak stack below main's frame while init/act/observe ran
 *   stack_total     peak stack from the top of RAM (includes startup + main)
 *   checksum        hash of the actions taken; must equal the host build's
 *
 * Peak working memory of the learner := agent_bytes + arena_bytes + stack_learner.
 *
 * Stack measurement = "stack painting": fill the unused RAM between the end of
 * .bss and the current stack pointer with a known pattern, run the workload,
 * then find the lowest address whose pattern was overwritten.
 *
 * Instruction counts are NOT measured here: scripts/measure_m4.py runs this
 * firmware twice with different BENCH_STEPS under a QEMU instruction-counting
 * plugin and takes the difference, which cancels the startup cost.
 *
 * The workload (bench_workload.h) feeds SYNTHETIC observations and rewards.
 */
#include <stdint.h>
#include "tinyrl.h"
#include "semihost.h"
#include "bench_cfg.h"   /* BENCH_CFG, ARENA_BYTES, BENCH_STEPS, BENCH_MODE */
#include "bench_workload.h"

#define PAINT 0xA5A5A5A5u

extern uint32_t _ebss, _estack;

static tr_agent_t ag;
static uint8_t arena[ARENA_BYTES] __attribute__((aligned(4)));
static const tr_cfg_t cfg = BENCH_CFG;

__attribute__((noinline)) static int run_init(void)
{
    return tr_init(&ag, &cfg, arena, sizeof(arena), 1u);
}

__attribute__((noinline)) static uint32_t run_bench(void)
{
    return bench_run(&ag, &cfg, BENCH_STEPS, BENCH_MODE);
}

int main(void)
{
    uint32_t sp;
    __asm__ volatile("mov %0, sp" : "=r"(sp));
    for (uint32_t *p = &_ebss; p < (uint32_t *)(sp - 64u); p++) *p = PAINT;

    int rc = run_init();
    uint32_t chk = rc == TR_OK ? run_bench() : 0u;

    uint32_t *low = &_ebss;
    while (low < (uint32_t *)sp && *low == PAINT) low++;

    sh_kv("init_ok", rc == TR_OK);
    sh_kv("agent_bytes", (uint32_t)sizeof(tr_agent_t));
    sh_kv("arena_bytes", (uint32_t)tr_mem_bytes(&cfg));
    sh_kv("arena_static", (uint32_t)sizeof(arena));
    sh_kv("stack_learner", sp - (uint32_t)low);
    sh_kv("stack_total", (uint32_t)&_estack - (uint32_t)low);
    sh_kv("steps", BENCH_STEPS);
    sh_kv("mode", BENCH_MODE);
    sh_kv("checksum", chk);
    return rc == TR_OK ? 0 : 1;
}
