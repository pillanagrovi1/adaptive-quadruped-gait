/*
 * corridor_run.c - run tabular Q-learning on the corridor and print a trace.
 *
 * Usage: corridor_run [episodes] [seed] [eps_q16] [n_cells]
 * Output (one line per episode):  ep <k> steps <n>
 * then the final table:           Q <cell> <q_left_raw> <q_right_raw>
 * Raw values are the int32 Q16.16 representation, so Python can compare
 * them bit for bit (see python/tests/test_corridor_gate.py).
 */
#include <stdio.h>
#include <stdlib.h>
#include "tinyrl.h"
#include "corridor.h"

#define MAX_STEPS 100

int main(int argc, char **argv)
{
    int episodes = argc > 1 ? atoi(argv[1]) : 20;
    uint32_t seed = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 10) : 1u;
    q16_t eps = argc > 3 ? (q16_t)atoi(argv[3]) : Q16(0.1);
    int n = argc > 4 ? atoi(argv[4]) : 6;

    tr_cfg_t cfg = {0};
    cfg.obs_dim = 1;
    cfg.n_actions = 2;
    cfg.obs_lo[0] = 0;
    cfg.obs_hi[0] = n * Q16_ONE;
    cfg.bins = n;
    cfg.alpha = Q16(0.5);
    cfg.gamma = Q16(0.9);

    static uint8_t arena[4096];
    static tr_agent_t ag;
    int rc = tr_init(&ag, &cfg, arena, sizeof(arena), seed);
    if (rc != TR_OK) { fprintf(stderr, "init failed %d\n", rc); return 1; }

    corridor_t env = { .n = n, .pos = 0 };
    for (int ep = 0; ep < episodes; ep++) {
        q16_t s = corridor_reset(&env);
        int steps = 0, done = 0;
        while (!done && steps < MAX_STEPS) {
            int a = tr_act(&ag, &s, eps);
            q16_t r;
            q16_t s2 = corridor_step(&env, a, &r, &done);
            tr_observe(&ag, &s, a, r, &s2, done);
            s = s2;
            steps++;
        }
        printf("ep %d steps %d\n", ep, steps);
    }
    for (int c = 0; c < n; c++) {
        q16_t s = c * Q16_ONE, q[2];
        tr_qvalues(&ag, &s, q);
        printf("Q %d %ld %ld\n", c, (long)q[0], (long)q[1]);
    }
    return 0;
}
