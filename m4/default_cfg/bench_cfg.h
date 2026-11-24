/*
 * Default benchmark config (the corridor table). Kept in its own folder:
 * a bench_cfg.h next to bench_main.c would shadow the per-config one,
 * because #include "..." searches the source file's folder first.
 * This is. scripts/measure_m4.py writes
 * its own bench_cfg.h per configuration into build/m4/<cfg_id>/ instead.
 */
#ifndef BENCH_CFG_H
#define BENCH_CFG_H

#define BENCH_CFG { .obs_dim = 1, .n_actions = 2, \
                    .obs_lo = {0}, .obs_hi = {6 * 65536}, \
                    .alpha = 32768, .gamma = 58982, .bins = 6 }
#define ARENA_BYTES 64
#define BENCH_STEPS 1000
#define BENCH_MODE  0

#endif
