/*
 * corridor.h - the Stage 1 toy task, small enough to solve by hand.
 *
 *   cells:   0   1   2   3   4   5
 *            S                   G       start S = 0, goal G = n-1
 *   actions: 0 = left, 1 = right (left at cell 0 stays put)
 *   reward:  +1 on entering the goal (episode ends), 0 otherwise
 *
 * With gamma = 0.9 the optimal values are Q*(c, right) = 0.9^(n-2-c).
 */
#ifndef CORRIDOR_H
#define CORRIDOR_H

#include "tr_fixed.h"

typedef struct { int n, pos; } corridor_t;

static inline q16_t corridor_reset(corridor_t *e)
{
    e->pos = 0;
    return (q16_t)(e->pos * Q16_ONE);
}

/* Returns the next observation; writes reward and done. */
static inline q16_t corridor_step(corridor_t *e, int a, q16_t *r, int *done)
{
    e->pos += a ? 1 : -1;
    if (e->pos < 0) e->pos = 0;
    *done = (e->pos == e->n - 1);
    *r = *done ? Q16_ONE : 0;
    return (q16_t)(e->pos * Q16_ONE);
}

#endif
