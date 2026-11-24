/*
 * tinyrl.h - an online Q-table learner that fits in a microcontroller's SRAM.
 *
 * Learning target (one-step Q-learning):
 *
 *     y = r + gamma * max_a' Q(s', a')      (y = r if s' is terminal)
 *     delta = y - Q(s, a)
 *     Q(s, a) <- Q(s, a) + alpha * delta
 *
 * Q(s, a) is stored in a uniform grid: one value per (cell, action).
 * Optional: Q(lambda) eligibility traces and an ObGD step-size cap (off by default).
 *
 * Memory model: NO malloc. The caller owns two blocks:
 *   - a tr_agent_t (fixed size, sizeof differs between host and M4)
 *   - an arena of tr_mem_bytes(cfg) bytes, carved up by tr_init()
 * On the microcontroller both are static arrays, so the linker sees them.
 *
 * All numbers crossing this API are Q16.16 (see tr_fixed.h).
 */
#ifndef TINYRL_H
#define TINYRL_H

#include <stddef.h>
#include <stdint.h>
#include "tr_fixed.h"

#define TR_MAX_OBS      12
#define TR_MAX_ACTIONS  32

enum {
    TR_OK          =  0,
    TR_ERR_CFG     = -1,   /* invalid configuration */
    TR_ERR_ARENA   = -2,   /* arena too small */
};

/*
 * Configuration. Every field is 32-bit so the Python ctypes mirror
 * (python/tinyrl/cbind.py) is trivial. Keep the field order in sync.
 */
typedef struct {
    int32_t obs_dim;                  /* 1..TR_MAX_OBS */
    int32_t n_actions;                /* 2..TR_MAX_ACTIONS */
    q16_t   obs_lo[TR_MAX_OBS];       /* observation bounds; values outside */
    q16_t   obs_hi[TR_MAX_OBS];       /* are clipped before use */
    q16_t   alpha;                    /* step size */
    q16_t   gamma;                    /* discount */
    int32_t bins;                     /* cells per dimension */
    q16_t   td_clip;                  /* |delta| clip, 0 = off */
    /* Traces and ObGD. All default to 0 = plain one-step Q-learning. */
    q16_t   lambda;                   /* Q(lambda) trace decay; >0 enables traces */
    q16_t   obgd_kappa;               /* ObGD step-size cap (Elsayed et al. 2024), 0 = off; typical 2.0 */
    int32_t round_mode;               /* 0 = round to nearest, 1 = stochastic rounding of updates */
    int32_t trace_cap;                /* sparse-trace entries kept, 0 = automatic (64) */
    int32_t watkins;                  /* traces: 1 = cut the trace after a non-greedy action (Watkins), 0 = never */
} tr_cfg_t;

/* Sparse eligibility trace: (index, value) pairs, for one-hot / few-hot features. */
typedef struct {
    uint32_t *idx;
    q16_t    *val;
    uint32_t  n, cap;
} tr_strace_t;

typedef struct {
    q16_t *table;                     /* n_cells * n_actions */
    uint32_t n_cells;
    tr_strace_t tr;                   /* used only when lambda/ObGD is on */
} tr_tab_t;

typedef struct {
    tr_cfg_t cfg;
    uint32_t rng;
    q16_t   *q;                       /* action values scratch, n_actions */
    uint32_t n_updates;
    uint32_t arena_used;
    uint32_t n_upd_terms;             /* weight-update products computed */
    uint32_t n_underflow;             /* ... of which were non-zero before rounding, zero after */
    tr_tab_t tab;
#ifndef TR_NO_DIAG
    /* Last, and absent from the M4 build, so its memory and instruction counts are untouched. */
    uint32_t n_saturated;             /* update terms whose product or weight write was clipped */
    uint32_t term_saturated;          /* the current term is already counted */
    uint32_t step_saturated;          /* streaming: the step shared by this update's terms was clipped */
#endif
} tr_agent_t;

/* Bytes of arena this configuration needs (0 if the config is invalid). */
size_t   tr_mem_bytes(const tr_cfg_t *cfg);
/* sizeof(tr_agent_t) on this build (Python needs it to allocate). */
size_t   tr_agent_sizeof(void);
/* Number of table entries (for logs). */
uint32_t tr_param_count(const tr_cfg_t *cfg);

int  tr_init(tr_agent_t *ag, const tr_cfg_t *cfg, void *arena, size_t arena_bytes, uint32_t seed);

/* Q(s, .) into out[n_actions]. */
void tr_qvalues(tr_agent_t *ag, const q16_t *obs, q16_t *out);
/* Greedy action, ties broken uniformly at random. */
int  tr_greedy(tr_agent_t *ag, const q16_t *obs);
/* Epsilon-greedy action; eps is Q16.16 in [0, 1]. */
int  tr_act(tr_agent_t *ag, const q16_t *obs, q16_t eps);
/* Learn from one transition. Returns number of updates performed (always 1). */
int  tr_observe(tr_agent_t *ag, const q16_t *s, int a, q16_t r, const q16_t *s2, int done);
/* Call at the start of every episode: clears eligibility traces (no-op without them).
 * Needed because a time-limit truncation is not passed to tr_observe as done. */
void tr_episode_start(tr_agent_t *ag);
/* Diagnostics: how many update products rounded to exactly zero (Q16.16 underflow). */
uint32_t tr_underflow_count(const tr_agent_t *ag);
uint32_t tr_update_terms(const tr_agent_t *ag);
/* ... and how many were clipped to the int32 range (always 0 with TR_NO_DIAG). */
uint32_t tr_saturation_count(const tr_agent_t *ag);
/* 1 if this config uses the streaming path (lambda > 0 or ObGD on). */
int  tr_is_stream(const tr_cfg_t *cfg);

#endif
