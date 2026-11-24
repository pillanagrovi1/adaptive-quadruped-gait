/*
 * test_core.c - unit tests for fixed-point maths and the table learner.
 * Plain asserts, no framework. Run: make test
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tinyrl.h"
#include "tr_rng.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static double f(q16_t x) { return x / 65536.0; }

static void test_fixed(void)
{
    CHECK(q16_mul(Q16(1.5), Q16(2.0)) == Q16(3.0), "1.5*2");
    CHECK(q16_mul(Q16(-1.5), Q16(2.0)) == Q16(-3.0), "-1.5*2");
    CHECK(q16_add(Q16_MAX, Q16(1)) == Q16_MAX, "saturating add");
    CHECK(q16_sub(Q16_MIN, Q16(1)) == Q16_MIN, "saturating sub");
    CHECK(fabs(f(q16_sqrt(Q16(2.0))) - sqrt(2.0)) < 1e-4, "sqrt 2 = %f", f(q16_sqrt(Q16(2.0))));
    CHECK(q16_mul3(Q16(0.001), Q16(0.01), Q16(1.0)) != 0, "mul3 keeps small products");
    for (int k = -50; k <= 50; k++) {
        double a = k * 0.37, b = k * -0.11;
        double got = f(q16_mul(Q16(a), Q16(b)));
        /* inputs are themselves rounded to 1/65536, so error ~ (|a|+|b|)/65536 */
        double tol = (fabs(a) + fabs(b) + 1.0) / 65536.0;
        CHECK(fabs(got - a * b) < tol, "mul %f*%f=%f got %f", a, b, a * b, got);
    }
}

static void base_cfg(tr_cfg_t *c, int d, int A)
{
    memset(c, 0, sizeof(*c));
    c->obs_dim = d; c->n_actions = A;
    for (int i = 0; i < d; i++) { c->obs_lo[i] = Q16(-1); c->obs_hi[i] = Q16(1); }
    c->alpha = Q16(0.1); c->gamma = Q16(0.9);
    c->bins = 8; c->td_clip = Q16(1.0);
}

/*
 * Regression test: a 1-step task where Q(s, a) should converge to a known
 * function f(s, a) = s0 * (a ? 1 : -1) * 0.5. The table must get close.
 */
static void test_regression(double tol)
{
    tr_cfg_t c; base_cfg(&c, 2, 2);
    static uint8_t arena[1 << 16];
    static tr_agent_t ag;
    size_t need = tr_mem_bytes(&c);
    CHECK(need > 0 && need <= sizeof(arena), "mem %zu", need);
    CHECK(tr_init(&ag, &c, arena, sizeof(arena), 7) == TR_OK, "init");
    uint32_t rng = 12345;
    for (int it = 0; it < 40000; it++) {
        q16_t s[2] = { (q16_t)((int32_t)(tr_rng_next(&rng) >> 15) - Q16_ONE),
                       (q16_t)((int32_t)(tr_rng_next(&rng) >> 15) - Q16_ONE) };
        int a = (int)(tr_rng_next(&rng) & 1);
        q16_t r = q16_mul(s[0], a ? Q16(0.5) : Q16(-0.5));
        tr_observe(&ag, s, a, r, s, 1);            /* terminal: y = r */
    }
    double err = 0; int n = 0;
    for (double x = -0.9; x <= 0.9; x += 0.3) {
        q16_t s[2] = { Q16(x), Q16(0.2) }, q[2];
        tr_qvalues(&ag, s, q);
        err += fabs(f(q[1]) - 0.5 * x) + fabs(f(q[0]) + 0.5 * x);
        n += 2;
    }
    err /= n;
    printf("  regression mean abs err %.4f (arena %u B)\n", err, ag.arena_used);
    CHECK(err < tol, "err %.4f >= %.4f", err, tol);
    CHECK(ag.arena_used == need, "arena_used %u != mem %zu", ag.arena_used, need);
}

static void test_bad_cfg(void)
{
    tr_cfg_t c; base_cfg(&c, 8, 2);
    c.bins = 100;                                  /* 100^8 cells: must be rejected */
    CHECK(tr_mem_bytes(&c) == 0, "huge table rejected");
    static uint8_t small[16];
    static tr_agent_t ag;
    base_cfg(&c, 2, 2);
    CHECK(tr_init(&ag, &c, small, sizeof(small), 1) == TR_ERR_ARENA, "small arena rejected");
}

/* Streaming path (ObGD, lambda = 0): the table must still fit the 1-step function. */
static void test_stream_regression(int round_mode, double tol)
{
    tr_cfg_t c; base_cfg(&c, 2, 2);
    c.alpha = Q16(1.0); c.obgd_kappa = Q16(2.0); c.lambda = 0; c.td_clip = 0;
    c.round_mode = round_mode;
    static uint8_t arena[1 << 16];
    static tr_agent_t ag;
    size_t need = tr_mem_bytes(&c);
    CHECK(tr_init(&ag, &c, arena, sizeof(arena), 11) == TR_OK, "stream init");
    uint32_t rng = 999;
    for (int it = 0; it < 40000; it++) {
        q16_t s[2] = { (q16_t)((int32_t)(tr_rng_next(&rng) >> 15) - Q16_ONE),
                       (q16_t)((int32_t)(tr_rng_next(&rng) >> 15) - Q16_ONE) };
        int a = (int)(tr_rng_next(&rng) & 1);
        q16_t r = q16_mul(s[0], a ? Q16(0.5) : Q16(-0.5));
        tr_episode_start(&ag);
        tr_observe(&ag, s, a, r, s, 1);
    }
    double err = 0; int n = 0;
    for (double x = -0.9; x <= 0.9; x += 0.3) {
        q16_t s[2] = { Q16(x), Q16(0.2) }, q[2];
        tr_qvalues(&ag, s, q);
        err += fabs(f(q[1]) - 0.5 * x) + fabs(f(q[0]) + 0.5 * x);
        n += 2;
    }
    err /= n;
    printf("  stream round %d: err %.4f, underflow %u/%u (arena %u B)\n",
           round_mode, err, tr_underflow_count(&ag), tr_update_terms(&ag), ag.arena_used);
    CHECK(err < tol, "stream round %d err %.4f >= %.4f", round_mode, err, tol);
    CHECK(ag.arena_used == need, "stream arena_used %u != mem %zu", ag.arena_used, need);
}

/* Q(lambda) (no Watkins cut) on the corridor: traces must reach the optimum too. */
static void test_qlambda_corridor(void)
{
    tr_cfg_t c; memset(&c, 0, sizeof(c));
    c.obs_dim = 1; c.n_actions = 2;
    c.obs_lo[0] = 0; c.obs_hi[0] = 6 * Q16_ONE;
    c.bins = 6;
    c.alpha = Q16(0.5); c.gamma = Q16(0.9); c.lambda = Q16(0.8);
    static uint8_t arena[1 << 14];
    static tr_agent_t ag;
    CHECK(tr_init(&ag, &c, arena, sizeof(arena), 5) == TR_OK, "qlambda init");
    for (int ep = 0; ep < 300; ep++) {
        int pos = 0, done = 0, steps = 0;
        tr_episode_start(&ag);
        while (!done && steps++ < 100) {
            /* cell centres, so no observation sits on a cell edge */
            q16_t s = pos * Q16_ONE + Q16_HALF;
            int a = tr_act(&ag, &s, Q16(0.2));
            pos += a ? 1 : -1; if (pos < 0) pos = 0;
            done = pos == 5;
            q16_t s2 = pos * Q16_ONE + Q16_HALF;
            tr_observe(&ag, &s, a, done ? Q16_ONE : 0, &s2, done);
        }
    }
    double worst = 0;
    for (int cell = 0; cell < 5; cell++) {
        q16_t s = cell * Q16_ONE + Q16_HALF, q[2];
        tr_qvalues(&ag, &s, q);
        double e = fabs(f(q[1]) - pow(0.9, 4 - cell));
        if (e > worst) worst = e;
    }
    printf("  Q(lambda=0.8) corridor: worst |Q - Q*| = %.4f\n", worst);
    CHECK(worst < 0.02, "Q(lambda) corridor worst err %.4f", worst);
}

/* Watkins cut: a non-greedy action must start a new trace. Two states, s0 -(a0)-> s1, then a
 * non-greedy a0 in s1 ends the episode with reward 1. With the cut Q(s0,a0) keeps its value; without it the
 * reward also flows back to (s0,a0): 0.225 -> 0.63 (alpha 0.5, gamma 0.9, lambda 0.9). */
static void test_watkins_cut(int watkins)
{
    tr_cfg_t c; memset(&c, 0, sizeof(c));
    c.obs_dim = 1; c.n_actions = 2;
    c.obs_lo[0] = 0; c.obs_hi[0] = Q16_ONE;
    c.bins = 2;
    c.alpha = Q16(0.5); c.gamma = Q16(0.9); c.lambda = Q16(0.9); c.watkins = watkins;
    static uint8_t arena[1 << 14];
    static tr_agent_t ag;
    CHECK(tr_init(&ag, &c, arena, sizeof(arena), 1) == TR_OK, "watkins init");
    q16_t s0 = Q16(0.25), s1 = Q16(0.75), q[2];
    tr_episode_start(&ag);
    tr_observe(&ag, &s1, 1, Q16_ONE, &s0, 1);       /* makes a1 greedy in s1 */
    tr_observe(&ag, &s0, 0, 0, &s1, 0);             /* (s0, a0) -> s1 */
    tr_qvalues(&ag, &s0, q);
    double before = f(q[0]);
    tr_observe(&ag, &s1, 0, Q16_ONE, &s0, 1);       /* a0 in s1 is not greedy */
    tr_qvalues(&ag, &s0, q);
    double after = f(q[0]), want = watkins ? before : 0.63;
    printf("  watkins=%d: Q(s0,a0) %.4f -> %.4f\n", watkins, before, after);
    CHECK(fabs(before - 0.225) < 1e-3 && fabs(after - want) < 1e-3,
          "watkins=%d: Q(s0,a0) %.4f -> %.4f, want %.4f", watkins, before, after, want);
}

/* Stochastic rounding keeps sub-LSB updates alive on average. */
static void test_stochastic_rounding(void)
{
    for (int mode = 0; mode <= 1; mode++) {
        tr_cfg_t c; base_cfg(&c, 1, 2);
        c.bins = 1; c.alpha = 2;                        /* alpha = 2/65536: tiny */
        c.round_mode = mode;
        static uint8_t arena[256];
        static tr_agent_t ag;
        tr_init(&ag, &c, arena, sizeof(arena), 3);
        q16_t s = 0;
        for (int k = 0; k < 20000; k++) tr_observe(&ag, &s, 0, Q16(0.1), &s, 1);
        q16_t q[2];
        tr_qvalues(&ag, &s, q);
        /* exact: Q -> 0.1 * (1 - (1-alpha)^20000) = 0.1 * (1 - e^-0.61) = 0.0457 */
        printf("  rounding mode %d: Q after 20000 tiny updates = %.4f (exact ~0.0457), underflow %u/%u\n",
               mode, f(q[0]), tr_underflow_count(&ag), tr_update_terms(&ag));
        if (mode == 0) CHECK(f(q[0]) == 0.0, "nearest rounding should lose every sub-LSB update");
        else CHECK(fabs(f(q[0]) - 0.0457) < 0.01, "stochastic rounding mean %.4f", f(q[0]));
    }
}

/* Saturation counter: one count per update term that was clipped. */
static void test_saturation(void)
{
    static uint8_t arena[8192];
    static tr_agent_t ag;
    for (int stream = 0; stream <= 1; stream++) {
        tr_cfg_t c; base_cfg(&c, 1, 2);
        c.bins = 1; c.td_clip = 0;
        c.alpha = Q16(1.5);
        if (stream) c.lambda = Q16(0.5);
        q16_t s = 0;
        CHECK(tr_init(&ag, &c, arena, sizeof(arena), 1) == TR_OK, "init");
        tr_observe(&ag, &s, 0, Q16(20000), &s, 1);     /* weight = 30000: in range */
        CHECK(tr_saturation_count(&ag) == 0, "stream %d: in range", stream);
        tr_observe(&ag, &s, 0, Q16(32000), &s, 1);     /* 30000 + 3000: the write clips */
        CHECK(tr_saturation_count(&ag) == 1, "stream %d: clipped write, got %u",
              stream, tr_saturation_count(&ag));
        c.alpha = Q16(2.0);
        tr_init(&ag, &c, arena, sizeof(arena), 1);
        tr_observe(&ag, &s, 0, Q16(20000), &s, 1);     /* 2 * 20000: the product clips */
        CHECK(tr_saturation_count(&ag) == 1, "stream %d: clipped product or step counts once, got %u",
              stream, tr_saturation_count(&ag));
    }
}

int main(void)
{
    test_fixed();
    test_regression(0.08);               /* bins=8: discretisation error ~0.06 */
    test_bad_cfg();
    test_stream_regression(0, 0.08);
    test_stream_regression(1, 0.08);
    test_qlambda_corridor();
    for (int w = 0; w <= 1; w++) test_watkins_cut(w);
    test_stochastic_rounding();
    test_saturation();
    printf("%s (%d failures)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}
