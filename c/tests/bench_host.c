/*
 * bench_host.c - the M4 benchmark workload compiled for the laptop.
 * Prints "checksum=<n>"; scripts/measure_m4.py checks it equals the M4's.
 * Build: gcc -I<cfg_dir> -Ic/include -Ic/src -Im4 c/tests/bench_host.c c/src/*.c
 */
#include <stdio.h>
#include <stdlib.h>
#include "tinyrl.h"
#include "bench_cfg.h"
#include "bench_workload.h"

int main(void)
{
    static const tr_cfg_t cfg = BENCH_CFG;
    static tr_agent_t ag;
    void *arena = malloc(ARENA_BYTES);
    int rc = tr_init(&ag, &cfg, arena, ARENA_BYTES, 1u);
    if (rc != TR_OK) { printf("init_ok=0\n"); return 1; }
    printf("checksum=%lu\n", (unsigned long)bench_run(&ag, &cfg, BENCH_STEPS, BENCH_MODE));
    free(arena);
    return 0;
}
