/*
 * tr_agent.c - config checks, arena layout, epsilon-greedy and the Q-learning
 * target. The table itself lives in tr_tabular.c.
 */
#include <string.h>
#include "tr_internal.h"

static int cfg_valid(const tr_cfg_t *c)
{
    if (!c) return 0;
    if (c->obs_dim < 1 || c->obs_dim > TR_MAX_OBS) return 0;
    if (c->n_actions < 2 || c->n_actions > TR_MAX_ACTIONS) return 0;
    for (int i = 0; i < c->obs_dim; i++)
        if (c->obs_hi[i] <= c->obs_lo[i]) return 0;
    if (c->gamma < 0 || c->gamma > Q16_ONE) return 0;
    if (c->alpha <= 0) return 0;
    if (c->bins < 1) return 0;
    if (c->lambda < 0 || c->lambda > Q16_ONE || c->obgd_kappa < 0) return 0;
    if (c->round_mode < 0 || c->round_mode > 1 || c->trace_cap < 0) return 0;
    if (c->watkins < 0 || c->watkins > 1) return 0;
    return 1;
}

size_t tr_mem_bytes(const tr_cfg_t *c)
{
    if (!cfg_valid(c)) return 0;
    size_t lm = tr_tab_mem(c);
    if (lm == 0) return 0;                                 /* table too large */
    return TR_ALIGN4(c->n_actions * sizeof(q16_t)) + lm;   /* q scratch + table */
}

size_t tr_agent_sizeof(void) { return sizeof(tr_agent_t); }
uint32_t tr_underflow_count(const tr_agent_t *ag) { return ag->n_underflow; }
uint32_t tr_update_terms(const tr_agent_t *ag) { return ag->n_upd_terms; }

uint32_t tr_saturation_count(const tr_agent_t *ag)
{
#ifndef TR_NO_DIAG
    return ag->n_saturated;
#else
    (void)ag;
    return 0;
#endif
}

uint32_t tr_param_count(const tr_cfg_t *c)
{
    if (!cfg_valid(c)) return 0;
    uint64_t n = (uint64_t)c->n_actions;
    for (int i = 0; i < c->obs_dim; i++) n *= (uint64_t)c->bins;
    return n > 0xFFFFFFFFu ? 0u : (uint32_t)n;
}

int tr_init(tr_agent_t *ag, const tr_cfg_t *cfg, void *arena, size_t arena_bytes, uint32_t seed)
{
    if (!ag || !cfg_valid(cfg)) return TR_ERR_CFG;
    size_t need = tr_mem_bytes(cfg);
    if (need == 0) return TR_ERR_CFG;
    if (!arena || arena_bytes < need) return TR_ERR_ARENA;

    memset(ag, 0, sizeof(*ag));
    ag->cfg = *cfg;
    ag->rng = tr_rng_seed(seed);

    memset(arena, 0, need);
    uint8_t *cur = (uint8_t *)arena;
    ag->q = (q16_t *)tr_carve(&cur, cfg->n_actions * sizeof(q16_t));
    int rc = tr_tab_init(ag, &cur);
    ag->arena_used = (uint32_t)(cur - (uint8_t *)arena);
    return rc;
}

void tr_qvalues(tr_agent_t *ag, const q16_t *obs, q16_t *out)
{
    tr_tab_q(ag, obs, out);
}

int tr_greedy(tr_agent_t *ag, const q16_t *obs)
{
    const int A = ag->cfg.n_actions;
    tr_tab_q(ag, obs, ag->q);
    q16_t best = ag->q[0];
    int n_best = 1;
    for (int k = 1; k < A; k++) {
        if (ag->q[k] > best) { best = ag->q[k]; n_best = 1; }
        else if (ag->q[k] == best) n_best++;
    }
    if (n_best == 1) {
        for (int k = 0; k < A; k++) if (ag->q[k] == best) return k;
    }
    uint32_t pick = tr_rng_below(&ag->rng, (uint32_t)n_best);
    for (int k = 0; k < A; k++) {
        if (ag->q[k] == best) {
            if (pick == 0) return k;
            pick--;
        }
    }
    return 0; /* unreachable */
}

int tr_act(tr_agent_t *ag, const q16_t *obs, q16_t eps)
{
    if (tr_rng_unit_q16(&ag->rng) < eps)
        return (int)tr_rng_below(&ag->rng, (uint32_t)ag->cfg.n_actions);
    return tr_greedy(ag, obs);
}

/* The table's one-step write is checked from outside, so that tr_tabular.c holds only the
 * algorithm: a value left on the int32 limit counts as clipped. Reads only; no RNG. */
static inline void count_table_saturation(tr_agent_t *ag, const q16_t *s, int a)
{
#ifndef TR_NO_DIAG
    tr_tab_q(ag, s, ag->q);
    if (ag->q[a] == Q16_MAX || ag->q[a] == Q16_MIN) tr_mark_saturated(ag);
#else
    (void)ag; (void)s; (void)a;
#endif
}

/* y = r + gamma * max_a' Q(s', a'), or r if s' is terminal. */
static q16_t target(tr_agent_t *ag, q16_t r, const q16_t *s2, int done)
{
    const tr_cfg_t *c = &ag->cfg;
    if (done) return r;
    tr_tab_q(ag, s2, ag->q);
    q16_t m = ag->q[0];
    for (int k = 1; k < c->n_actions; k++) if (ag->q[k] > m) m = ag->q[k];
    return q16_add(r, q16_mul(c->gamma, m));
}

/* One-step Q-learning update on one transition. */
static void learn_one(tr_agent_t *ag, const q16_t *s, int a, q16_t r, const q16_t *s2, int done)
{
    tr_tab_update(ag, s, a, target(ag, r, s2, done));
    count_table_saturation(ag, s, a);
    ag->n_updates++;
}

int tr_is_stream(const tr_cfg_t *cfg) { return tr_stream_on(cfg); }

void tr_episode_start(tr_agent_t *ag)
{
    if (tr_stream_on(&ag->cfg)) tr_tab_clear(ag);
}

/*
 * Streaming update: Q(lambda) WITHOUT the Watkins cut, with an ObGD step-size cap
 * (Elsayed, Vasan & Mahmood 2024, "Streaming Deep RL Finally Works").
 *
 *   y      = r + gamma * max_a' Q(s', a')              (r if terminal): the SAME target
 *   e      <- gamma*lambda * e + grad Q(s, a)          watkins = 1: e <- grad Q(s, a) if Q(s, a) < max Q(s, .)
 *                                                     watkins = 0 (default): never cut
 *   delta  = y - Q(s, a)
 *   ObGD:  dbar = max(|delta|, 1);  alpha_eff = alpha * min(1, 1 / (kappa * dbar * ||e||_1))
 *   w      <- w + alpha_eff * delta * e
 *   e      <- 0 at terminal states and at episode start (tr_episode_start)
 *
 * With lambda = 0 and kappa = 0 this is exactly the one-step update.
 */
static void learn_stream(tr_agent_t *ag, const q16_t *s, int a, q16_t r, const q16_t *s2, int done)
{
    const tr_cfg_t *c = &ag->cfg;
    q16_t y = target(ag, r, s2, done);
    q16_t gl = q16_mul(c->gamma, c->lambda);
    if (c->watkins && gl != 0) {
        /* Watkins cut: a non-greedy a (strictly below the max; ties count as greedy, no RNG) starts a new trace. */
        tr_tab_q(ag, s, ag->q);
        q16_t m = ag->q[0];
        for (int k = 1; k < c->n_actions; k++) if (ag->q[k] > m) m = ag->q[k];
        if (ag->q[a] < m) gl = 0;
    }
    int64_t n1 = 0;
    q16_t qsa = tr_tab_trace(ag, s, a, gl, &n1);
    q16_t delta = tr_clip_delta(c, q16_sub(y, qsa));
    q16_t step = q16_mul(c->alpha, delta);
#ifndef TR_NO_DIAG
    int64_t unclipped = ((int64_t)c->alpha * delta + (1LL << 15)) >> 16;
    ag->step_saturated = (step != unclipped);       /* then every term made from it is clipped */
#endif
    int sh = 16;                      /* step is Q16.16 ... */
    if (c->obgd_kappa > 0) {
        q16_t ad = delta < 0 ? -delta : delta;
        q16_t dbar = ad > Q16_ONE ? ad : Q16_ONE;
        int64_t denom = ((int64_t)q16_mul(c->obgd_kappa, dbar) * n1) >> 16;   /* Q16.16 */
        if (denom > Q16_ONE) {
            /* ... unless ObGD shrinks it: then keep 14 extra fraction bits (Q2.30), so a
             * step of e.g. 1e-4 is not rounded to 6 LSBs (or 0) before it multiplies e.
             * One division per update, never per weight. */
            int64_t step30 = ((int64_t)q16_mul(c->alpha, delta) * (1LL << 30)) / denom;
            if (step30 > -(1LL << 31) && step30 < (1LL << 31)) { step = (q16_t)step30; sh = 30; }
            else step = q16_sat(((int64_t)step * 65536) / denom);
        }
    }
    if (step != 0) tr_tab_apply(ag, step, sh);
    if (done) tr_episode_start(ag);
    ag->n_updates++;
}

int tr_observe(tr_agent_t *ag, const q16_t *s, int a, q16_t r, const q16_t *s2, int done)
{
    if (tr_stream_on(&ag->cfg)) learn_stream(ag, s, a, r, s2, done);
    else learn_one(ag, s, a, r, s2, done);
    return 1;
}
