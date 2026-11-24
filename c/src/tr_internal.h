/* tr_internal.h - shared helpers between the agent core and the learners. */
#ifndef TR_INTERNAL_H
#define TR_INTERNAL_H

#include "tinyrl.h"
#include "tr_rng.h"

#define TR_ALIGN4(n) (((size_t)(n) + 3u) & ~(size_t)3u)

/* ---- The table learner (tr_tabular.c) ----
 * One-step path: tr_tab_q reads Q(s, .), tr_tab_update moves Q(s, a) towards y.
 * Streaming path (Q(lambda) without the Watkins cut, + ObGD), used when tr_is_stream(cfg):
 * tr_tab_trace: reads Q(s, ·) into ag->q, then
 *          e <- decay * e + grad_w Q(s, a); writes ||e||_1 to *norm1; returns Q(s, a).
 * tr_tab_apply: w <- w + (step * e) >> sh   (sh = 16: step is Q16.16; sh = 30: step is Q2.30,
 *          used by ObGD when the effective step is too small for Q16.16).
 * tr_tab_clear: e <- 0.
 */
size_t tr_tab_mem(const tr_cfg_t *c);
int    tr_tab_init(tr_agent_t *ag, uint8_t **cursor);
void   tr_tab_q(tr_agent_t *ag, const q16_t *obs, q16_t *out);
void   tr_tab_update(tr_agent_t *ag, const q16_t *obs, int a, q16_t y);
q16_t  tr_tab_trace(tr_agent_t *ag, const q16_t *obs, int a, q16_t decay, int64_t *norm1);
void   tr_tab_apply(tr_agent_t *ag, q16_t step, int sh);
void   tr_tab_clear(tr_agent_t *ag);

/* ---- Sparse trace (tr_trace.c) ---- */
size_t  tr_strace_mem(uint32_t cap);
void    tr_strace_init(tr_strace_t *t, uint8_t **cursor, uint32_t cap);
void    tr_strace_decay(tr_strace_t *t, q16_t decay);
void    tr_strace_add(tr_strace_t *t, uint32_t index, q16_t v);
int64_t tr_strace_norm1(const tr_strace_t *t);
void    tr_strace_apply(tr_agent_t *ag, const tr_strace_t *t, q16_t *w, q16_t step, int sh);
static inline void tr_strace_clear(tr_strace_t *t) { t->n = 0; }

static inline int tr_stream_on(const tr_cfg_t *c) { return c->lambda > 0 || c->obgd_kappa > 0; }

/* Clip the TD error if the config asks for it. */
static inline q16_t tr_clip_delta(const tr_cfg_t *c, q16_t d)
{
    if (c->td_clip > 0) return q16_clip(d, -c->td_clip, c->td_clip);
    return d;
}

/*
 * a * b with the configured rounding of LEARNING UPDATES.
 * round_mode 0: round to nearest (same as q16_mul).
 * round_mode 1: stochastic rounding (Gupta et al. 2015): add a uniform random
 *   fraction in [0, 1) LSB before truncating, so E[result] = a*b exactly and
 *   updates smaller than one LSB survive on average instead of vanishing.
 */
/* Underflow and saturation diagnostics. The M4 build defines TR_NO_DIAG so that instruction
 * counts measure the learner, not the bookkeeping; the counters never affect
 * any action, so host and M4 still agree bit for bit. */
static inline void tr_mark_saturated(tr_agent_t *ag)
{
#ifndef TR_NO_DIAG
    if (!ag->term_saturated) { ag->term_saturated = 1; ag->n_saturated++; }
#else
    (void)ag;
#endif
}

/* v = the rounded product, before saturation. */
static inline q16_t tr_count_upd(tr_agent_t *ag, int64_t p, int64_t v)
{
    q16_t r = q16_sat(v);
#ifndef TR_NO_DIAG
    ag->n_upd_terms++;
    if (r == 0 && p != 0) ag->n_underflow++;
    ag->term_saturated = 0;
    if (r != v || ag->step_saturated) tr_mark_saturated(ag);
#else
    (void)ag; (void)p;
#endif
    return r;
}

/* w + the current update term, saturated (and counted). */
static inline q16_t tr_add_upd(tr_agent_t *ag, q16_t w, q16_t d)
{
#ifndef TR_NO_DIAG
    int64_t v = (int64_t)w + d;
    if (v != q16_sat(v)) tr_mark_saturated(ag);
#else
    (void)ag;
#endif
    return q16_add(w, d);
}

static inline q16_t tr_mul_upd(tr_agent_t *ag, q16_t a, q16_t b)
{
    int64_t p = (int64_t)a * (int64_t)b;
    if (ag->cfg.round_mode == 1)
        return tr_count_upd(ag, p, (p + (int64_t)(tr_rng_next(&ag->rng) >> 16)) >> 16);
    return tr_count_upd(ag, p, (p + (1LL << 15)) >> 16);
}

/* (a * b) >> sh with the configured rounding; sh = 16 is tr_mul_upd. */
static inline q16_t tr_mul_upd_sh(tr_agent_t *ag, q16_t a, q16_t b, int sh)
{
    int64_t p = (int64_t)a * (int64_t)b;
    if (ag->cfg.round_mode == 1) {
        int64_t r = (int64_t)(tr_rng_next(&ag->rng) >> (32 - sh));   /* uniform in [0, 2^sh) */
        return tr_count_upd(ag, p, (p + r) >> sh);
    }
    return tr_count_upd(ag, p, (p + (1LL << (sh - 1))) >> sh);
}

static inline void *tr_carve(uint8_t **cursor, size_t bytes)
{
    void *p = *cursor;
    *cursor += TR_ALIGN4(bytes);
    return p;
}

#endif
