/*
 * tr_tabular.c - Q-table over a uniform grid.
 *
 * cell(s) = sum_i bin_i(s_i) * bins^i,   bin_i = floor((s_i - lo_i) * bins / (hi_i - lo_i))
 * Memory: bins^obs_dim * n_actions * 4 bytes. Grows exponentially with obs_dim,
 * so the observation must stay small (a few IMU and contact features).
 *
 * Streaming path (lambda/ObGD): the gradient of Q(s,a) w.r.t. the table is
 * one-hot, so the trace is a short sparse list (tr_trace.c), default 64 entries.
 */
#include "tr_internal.h"

#define TAB_MAX_CELLS (1u << 22)
#define TAB_TRACE_DEFAULT 64u

static uint32_t n_cells(const tr_cfg_t *c)
{
    uint64_t n = 1;
    for (int i = 0; i < c->obs_dim; i++) {
        n *= (uint64_t)c->bins;
        if (n > TAB_MAX_CELLS) return 0;
    }
    return (uint32_t)n;
}

static uint32_t trace_cap(const tr_cfg_t *c)
{
    return c->trace_cap > 0 ? (uint32_t)c->trace_cap : TAB_TRACE_DEFAULT;
}

size_t tr_tab_mem(const tr_cfg_t *c)
{
    uint32_t n = n_cells(c);
    if (n == 0) return 0;
    size_t m = TR_ALIGN4((size_t)n * c->n_actions * sizeof(q16_t));
    if (tr_stream_on(c)) m += tr_strace_mem(trace_cap(c));
    return m;
}

int tr_tab_init(tr_agent_t *ag, uint8_t **cursor)
{
    const tr_cfg_t *c = &ag->cfg;
    ag->tab.n_cells = n_cells(c);
    ag->tab.table = (q16_t *)tr_carve(cursor, (size_t)ag->tab.n_cells * c->n_actions * sizeof(q16_t));
    if (tr_stream_on(c)) tr_strace_init(&ag->tab.tr, cursor, trace_cap(c));
    /* table starts at 0 (arena was zeroed) */
    return TR_OK;
}

static uint32_t cell_of(const tr_agent_t *ag, const q16_t *obs)
{
    const tr_cfg_t *c = &ag->cfg;
    uint32_t cell = 0, stride = 1;
    for (int i = 0; i < c->obs_dim; i++) {
        q16_t x = q16_clip(obs[i], c->obs_lo[i], c->obs_hi[i]);
        uint64_t num = (uint64_t)((int64_t)x - c->obs_lo[i]) * (uint32_t)c->bins;
        uint64_t den = (uint64_t)((int64_t)c->obs_hi[i] - c->obs_lo[i]);
        /* floor division; 32-bit UDIV (2-12 cycles on the M4) when the operands fit,
         * instead of a 64-bit library call. Same result either way. */
        int32_t b = (num >> 32) == 0 && (den >> 32) == 0
                  ? (int32_t)((uint32_t)num / (uint32_t)den)
                  : (int32_t)(num / den);
        if (b >= c->bins) b = c->bins - 1;         /* x == hi lands in the last bin */
        cell += (uint32_t)b * stride;
        stride *= (uint32_t)c->bins;
    }
    return cell;
}

void tr_tab_q(tr_agent_t *ag, const q16_t *obs, q16_t *out)
{
    const int A = ag->cfg.n_actions;
    const q16_t *row = &ag->tab.table[cell_of(ag, obs) * A];
    for (int k = 0; k < A; k++) out[k] = row[k];
}

void tr_tab_update(tr_agent_t *ag, const q16_t *obs, int a, q16_t y)
{
    const tr_cfg_t *c = &ag->cfg;
    q16_t *qa = &ag->tab.table[cell_of(ag, obs) * c->n_actions + a];
    q16_t delta = tr_clip_delta(c, q16_sub(y, *qa));
    *qa = q16_add(*qa, tr_mul_upd(ag, c->alpha, delta));
}

/* ---- streaming path ---- */

q16_t tr_tab_trace(tr_agent_t *ag, const q16_t *obs, int a, q16_t decay, int64_t *norm1)
{
    const int A = ag->cfg.n_actions;
    uint32_t base = cell_of(ag, obs) * A;
    for (int k = 0; k < A; k++) ag->q[k] = ag->tab.table[base + k];
    tr_strace_t *t = &ag->tab.tr;
    if (decay == 0) tr_strace_clear(t); else tr_strace_decay(t, decay);
    tr_strace_add(t, base + (uint32_t)a, Q16_ONE);
    *norm1 = tr_strace_norm1(t);
    return ag->q[a];
}

void tr_tab_apply(tr_agent_t *ag, q16_t step, int sh) { tr_strace_apply(ag, &ag->tab.tr, ag->tab.table, step, sh); }
void tr_tab_clear(tr_agent_t *ag) { tr_strace_clear(&ag->tab.tr); }
