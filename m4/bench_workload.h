/*
 * bench_workload.h - the benchmark workload, shared by the M4 firmware
 * (m4/bench_main.c) and a host build (c/tests/bench_host.c).
 *
 * Both builds print the same checksum of the actions taken iff the learner
 * behaves bit-identically on x86-64 and on the Cortex-M4. measure_m4.py
 * compares them, so host learning curves are known to describe the code that
 * runs on the chip.
 */
#ifndef BENCH_WORKLOAD_H
#define BENCH_WORKLOAD_H

#include <stdint.h>
#include "tinyrl.h"
#include "tr_rng.h"

static void bench_synth_obs(const tr_cfg_t *c, uint32_t *rng, q16_t *o)
{
    for (int i = 0; i < c->obs_dim; i++) {
        uint32_t range = (uint32_t)(c->obs_hi[i] - c->obs_lo[i]);
        o[i] = c->obs_lo[i] + (q16_t)(tr_rng_next(rng) % range);
    }
}

/* mode 0: act + observe (learning);  mode 1: greedy only (inference) */
static uint32_t bench_run(tr_agent_t *ag, const tr_cfg_t *c, uint32_t steps, int mode)
{
    uint32_t rng = 0xC0FFEEu, chk = 2166136261u;
    q16_t s[TR_MAX_OBS], s2[TR_MAX_OBS];
    bench_synth_obs(c, &rng, s);
    for (uint32_t k = 0; k < steps; k++) {
        int a;
        if (mode == 0) {
            a = tr_act(ag, s, Q16_ONE / 10);
            bench_synth_obs(c, &rng, s2);
            q16_t r = (q16_t)(tr_rng_next(&rng) & 0xFFFF) - Q16_HALF;   /* [-0.5, 0.5) */
            int done = (k % 50u) == 49u;
            tr_observe(ag, s, a, r, s2, done);
            for (int i = 0; i < c->obs_dim; i++) s[i] = s2[i];
        } else {
            a = tr_greedy(ag, s);
            bench_synth_obs(c, &rng, s);
        }
        chk = (chk ^ (uint32_t)a) * 16777619u;
    }
    return chk;
}

#endif
