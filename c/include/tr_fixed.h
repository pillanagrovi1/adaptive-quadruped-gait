/*
 * tr_fixed.h - Q16.16 fixed-point arithmetic.
 *
 * A q16_t holds a real number x as round(x * 65536) in an int32.
 * Range: about -32768 .. +32767.99998, resolution 1/65536 (~1.5e-5).
 *
 * Every operation here has an exact mirror in python/tinyrl/ref/fixed.py.
 * If you change one, change the other, or the corridor gate test fails.
 *
 * Rounding rule (used everywhere): add half an LSB, then arithmetic shift
 * right. That is "round half towards +infinity". GCC on x86 and ARM both
 * implement >> on negative signed ints as an arithmetic shift.
 */
#ifndef TR_FIXED_H
#define TR_FIXED_H

#include <stdint.h>

typedef int32_t q16_t;

#define Q16_ONE   ((q16_t)65536)
#define Q16_HALF  ((q16_t)32768)
#define Q16_MAX   ((q16_t)INT32_MAX)
#define Q16_MIN   ((q16_t)INT32_MIN)

/* Compile-time conversion for constants only (never in a hot loop). */
#define Q16(x)    ((q16_t)((x) >= 0 ? (x) * 65536.0 + 0.5 : (x) * 65536.0 - 0.5))

static inline q16_t q16_sat(int64_t v)
{
    if (v > INT32_MAX) return Q16_MAX;
    if (v < INT32_MIN) return Q16_MIN;
    return (q16_t)v;
}

static inline q16_t q16_add(q16_t a, q16_t b) { return q16_sat((int64_t)a + b); }
static inline q16_t q16_sub(q16_t a, q16_t b) { return q16_sat((int64_t)a - b); }

/* a * b, both Q16.16 -> Q16.16, rounded, saturated. */
static inline q16_t q16_mul(q16_t a, q16_t b)
{
    int64_t p = (int64_t)a * (int64_t)b;
    return q16_sat((p + (1LL << 15)) >> 16);
}

/* a * b * c in one rounding step (keeps tiny learning-rate products alive). */
static inline q16_t q16_mul3(q16_t a, q16_t b, q16_t c)
{
    /* |a*b| < 2^62 for |a|,|b| < 2^31; we shift first to stay in int64. */
    int64_t ab = (int64_t)a * (int64_t)b;          /* Q32.32 */
    int64_t ab16 = (ab + (1LL << 15)) >> 16;       /* Q16.16 in int64, unsaturated */
    int64_t p = ab16 * (int64_t)c;                 /* Q32.32 */
    return q16_sat((p + (1LL << 15)) >> 16);
}

/* Accumulator helpers for dot products: sum Q32.32 products, shift once. */
static inline q16_t q16_from_acc(int64_t acc)
{
    return q16_sat((acc + (1LL << 15)) >> 16);
}

static inline q16_t q16_clip(q16_t x, q16_t lo, q16_t hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* Integer square root of a 64-bit value (floor). */
static inline uint32_t tr_isqrt64(uint64_t x)
{
    uint64_t r = 0, bit = 1ULL << 62;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else              { r >>= 1; }
        bit >>= 2;
    }
    return (uint32_t)r;
}

/* sqrt of a non-negative Q16.16 value, result Q16.16. */
static inline q16_t q16_sqrt(q16_t x)
{
    if (x <= 0) return 0;
    return (q16_t)tr_isqrt64((uint64_t)x << 16);
}

#endif
