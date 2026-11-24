"""Stage 1 reference: Q-learning on the corridor, twice.

1. `run_float`  - the textbook algorithm in floating point. Use it to check
                  your hand calculation and to see the values converge.
2. `run_q16`    - the same algorithm in Q16.16, mirroring the C code line by
                  line (cell_of, epsilon-greedy with random tie-break, update).
                  The gate test requires the C trace to equal this one exactly.

Update rule (both):
    y     = r                          if s' is terminal
    y     = r + gamma * max_a' Q(s',a')  otherwise
    Q(s,a) <- Q(s,a) + alpha * (y - Q(s,a))
"""
from __future__ import annotations

import random

from . import fixed as fx

MAX_STEPS = 100


def corridor_step(pos: int, a: int, n: int):
    pos = max(0, pos + (1 if a else -1))
    done = pos == n - 1
    return pos, (1.0 if done else 0.0), done


def optimal_q(n: int = 6, gamma: float = 0.9):
    """Q*(c, right) = gamma^(n-2-c); Q*(c, left) = gamma * V*(max(c-1,0))."""
    v = [gamma ** (n - 2 - c) for c in range(n - 1)] + [0.0]
    q = []
    for c in range(n - 1):
        left = gamma * v[max(c - 1, 0)]
        right = 1.0 if c + 1 == n - 1 else gamma * v[c + 1]
        q.append((left, right))
    q.append((0.0, 0.0))
    return q


def run_float(episodes=20, alpha=0.5, gamma=0.9, eps=0.1, n=6, seed=1, verbose=False,
              always_right=False):
    """always_right=True replays the paper exercise: the agent always moves right
    (a fixed behaviour policy) while Q-learning updates as usual."""
    rng = random.Random(seed)
    Q = [[0.0, 0.0] for _ in range(n)]
    steps_log = []
    for ep in range(episodes):
        s, steps, done = 0, 0, False
        while not done and steps < MAX_STEPS:
            if always_right:
                a = 1
            elif rng.random() < eps:
                a = rng.randrange(2)
            else:
                best = max(Q[s])
                a = rng.choice([k for k in range(2) if Q[s][k] == best])
            s2, r, done = corridor_step(s, a, n)
            y = r if done else r + gamma * max(Q[s2])
            if verbose:
                print(f"ep{ep} s={s} a={a} r={r} s'={s2} y={y:.4f} "
                      f"Q[{s}][{a}]: {Q[s][a]:.4f} -> {Q[s][a] + alpha * (y - Q[s][a]):.4f}")
            Q[s][a] += alpha * (y - Q[s][a])
            s, steps = s2, steps + 1
        steps_log.append(steps)
    return Q, steps_log


class TabularQ16:
    """Line-by-line mirror of tr_tabular.c + the shared parts of tr_agent.c."""

    def __init__(self, n_cells, n_actions, lo, hi, alpha, gamma, seed):
        self.bins, self.A = n_cells, n_actions
        self.lo, self.hi = lo, hi
        self.alpha, self.gamma = alpha, gamma
        self.rng = fx.XorShift32(seed)
        self.table = [[0] * n_actions for _ in range(n_cells)]

    def cell_of(self, x):
        x = fx.clip(x, self.lo, self.hi)
        b = ((x - self.lo) * self.bins) // (self.hi - self.lo)
        return min(b, self.bins - 1)

    def greedy(self, x):
        row = self.table[self.cell_of(x)]
        best = max(row)
        ties = [k for k in range(self.A) if row[k] == best]
        if len(ties) == 1:
            return ties[0]
        return ties[self.rng.below(len(ties))]

    def act(self, x, eps):
        if self.rng.unit_q16() < eps:
            return self.rng.below(self.A)
        return self.greedy(x)

    def observe(self, s, a, r, s2, done):
        y = r
        if not done:
            y = fx.add(r, fx.mul(self.gamma, max(self.table[self.cell_of(s2)])))
        c = self.cell_of(s)
        q = self.table[c][a]
        self.table[c][a] = fx.add(q, fx.mul(self.alpha, fx.sub(y, q)))


def run_q16(episodes=20, seed=1, eps=None, n=6):
    """Returns (steps per episode, final table as raw int32), like corridor_run."""
    eps = fx.q16(0.1) if eps is None else eps
    ag = TabularQ16(n, 2, 0, n * fx.ONE, fx.q16(0.5), fx.q16(0.9), seed)
    steps_log = []
    for _ in range(episodes):
        pos, steps, done = 0, 0, False
        s = pos * fx.ONE
        while not done and steps < MAX_STEPS:
            a = ag.act(s, eps)
            pos, r, done = corridor_step(pos, a, n)
            s2, rq = pos * fx.ONE, (fx.ONE if done else 0)
            ag.observe(s, a, rq, s2, done)
            s, steps = s2, steps + 1
        steps_log.append(steps)
    return steps_log, [list(row) for row in ag.table]


if __name__ == "__main__":
    print("Paper exercise: alpha 0.5, gamma 0.9, always move right, 3 episodes\n")
    Q, steps = run_float(episodes=3, verbose=True, always_right=True)
    print("\nSteps per episode:", steps)
    print("Q(c, right) after 3 episodes:", [round(q[1], 4) for q in Q])
    print("Optimal Q*(c, right):", [round(q[1], 4) for q in optimal_q()])
