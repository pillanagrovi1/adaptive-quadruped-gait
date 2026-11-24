/*
 * tr_rng.h - xorshift32 pseudo-random generator (Marsaglia 2003).
 * Tiny (4 bytes of state), fast on Cortex-M4, and easy to mirror in Python.
 * Not cryptographic. Good enough for epsilon-greedy and random tie-breaks.
 */
#ifndef TR_RNG_H
#define TR_RNG_H

#include <stdint.h>

static inline uint32_t tr_rng_seed(uint32_t seed)
{
    return seed ? seed : 0x9E3779B9u;   /* state must never be 0 */
}

static inline uint32_t tr_rng_next(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/* Uniform integer in [0, n). Slight modulo bias, irrelevant for n << 2^32. */
static inline uint32_t tr_rng_below(uint32_t *s, uint32_t n)
{
    return tr_rng_next(s) % n;
}

/* Uniform Q16.16 in [0, 1). */
static inline int32_t tr_rng_unit_q16(uint32_t *s)
{
    return (int32_t)(tr_rng_next(s) >> 16);
}

#endif
