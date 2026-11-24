/*
 * tr_trace.c - sparse eligibility traces for the table learner.
 *
 * The table's gradient is one-hot, so a trace has
 * only a few non-zero entries at a time. Storing (index, value) pairs costs
 * cap * 8 bytes instead of one value per weight.
 *
 *   decay:  every value *= gamma*lambda. In Q16.16 a value of 1.0 decays to 0
 *           after about 47 steps at gamma*lambda = 0.79, and zero entries are
 *           dropped, so the list stays short on its own.
 *   add:    if the index is present, add to it; otherwise append; if full,
 *           replace the smallest entry (the one with least influence).
 */
#include "tr_internal.h"

size_t tr_strace_mem(uint32_t cap)
{
    return TR_ALIGN4((size_t)cap * sizeof(uint32_t)) + TR_ALIGN4((size_t)cap * sizeof(q16_t));
}

void tr_strace_init(tr_strace_t *t, uint8_t **cursor, uint32_t cap)
{
    t->idx = (uint32_t *)tr_carve(cursor, (size_t)cap * sizeof(uint32_t));
    t->val = (q16_t *)tr_carve(cursor, (size_t)cap * sizeof(q16_t));
    t->n = 0;
    t->cap = cap;
}

void tr_strace_decay(tr_strace_t *t, q16_t decay)
{
    uint32_t k = 0;
    for (uint32_t i = 0; i < t->n; i++) {
        q16_t v = q16_mul(t->val[i], decay);
        if (v != 0) { t->idx[k] = t->idx[i]; t->val[k] = v; k++; }
    }
    t->n = k;
}

void tr_strace_add(tr_strace_t *t, uint32_t index, q16_t v)
{
    for (uint32_t i = 0; i < t->n; i++)
        if (t->idx[i] == index) { t->val[i] = q16_add(t->val[i], v); return; }
    if (t->n < t->cap) {
        t->idx[t->n] = index;
        t->val[t->n] = v;
        t->n++;
        return;
    }
    uint32_t m = 0;
    for (uint32_t i = 1; i < t->n; i++)
        if ((t->val[i] < 0 ? -t->val[i] : t->val[i]) < (t->val[m] < 0 ? -t->val[m] : t->val[m])) m = i;
    t->idx[m] = index;
    t->val[m] = v;
}

int64_t tr_strace_norm1(const tr_strace_t *t)
{
    int64_t s = 0;
    for (uint32_t i = 0; i < t->n; i++) s += t->val[i] < 0 ? -(int64_t)t->val[i] : t->val[i];
    return s;
}

void tr_strace_apply(tr_agent_t *ag, const tr_strace_t *t, q16_t *w, q16_t step, int sh)
{
    for (uint32_t i = 0; i < t->n; i++) {
        q16_t *wi = &w[t->idx[i]];
        *wi = tr_add_upd(ag, *wi, tr_mul_upd_sh(ag, step, t->val[i], sh));
    }
}
