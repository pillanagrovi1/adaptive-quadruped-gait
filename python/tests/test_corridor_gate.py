"""Stage 1 gate: the C learner must reproduce the Python Q16.16 reference
bit for bit, and both must approach the analytic optimum.
"""
import subprocess
from pathlib import Path

import pytest

from tinyrl.ref import corridor_ref as ref
from tinyrl.ref import fixed as fx

ROOT = Path(__file__).resolve().parents[2]
BIN = ROOT / "build" / "corridor_run"


def run_c(episodes, seed, eps, n=6):
    out = subprocess.run([str(BIN), str(episodes), str(seed), str(eps), str(n)],
                         check=True, capture_output=True, text=True).stdout
    steps, table = [], []
    for line in out.splitlines():
        parts = line.split()
        if parts[0] == "ep":
            steps.append(int(parts[3]))
        elif parts[0] == "Q":
            table.append([int(parts[2]), int(parts[3])])
    return steps, table


@pytest.mark.skipif(not BIN.exists(), reason="run `make` first")
@pytest.mark.parametrize("seed", [1, 2, 3, 42, 1234])
@pytest.mark.parametrize("eps", [0.0, 0.1, 0.5])
def test_c_matches_python_bit_exact(seed, eps):
    e = fx.q16(eps)
    c_steps, c_table = run_c(30, seed, e)
    p_steps, p_table = ref.run_q16(30, seed, e)
    assert c_steps == p_steps
    assert c_table == p_table


def test_float_converges_to_optimum():
    Q, _ = ref.run_float(episodes=300, eps=0.2, seed=0)
    opt = ref.optimal_q()
    for c in range(5):
        assert abs(Q[c][1] - opt[c][1]) < 1e-3


@pytest.mark.skipif(not BIN.exists(), reason="run `make` first")
def test_q16_close_to_float():
    _, table = run_c(300, 5, fx.q16(0.2))
    opt = ref.optimal_q()
    for c in range(5):
        assert abs(fx.to_float(table[c][1]) - opt[c][1]) < 1e-3
