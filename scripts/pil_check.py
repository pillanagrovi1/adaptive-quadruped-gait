#!/usr/bin/env python3
"""Processor-in-the-loop check: learning ON the (emulated) Cortex-M4 must reproduce
learning on the laptop bit for bit.

Starts the PIL firmware (m4/pil) in QEMU with its UART on a TCP socket, then runs
the same corridor episodes twice per learner: once with the host library, once with
the learner on the chip (the laptop only steps the corridor and relays observations).
Every action must be identical. Not in CI yet (Stage 5 will use it with the robot).

    python3 scripts/pil_check.py                  # QEMU
    python3 scripts/pil_check.py --link serial:/dev/ttyACM0 --no-qemu   # a flashed Nucleo board

On a real board this also records cycles per act() and observe() call, the
first timing numbers the project is allowed to claim (on that board, that clock).
Output: results/m4/pil_check.json
"""
import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from tinyrl.cbind import Agent  # noqa: E402
from tinyrl.pil import PilAgent  # noqa: E402

LEARNERS = {
    "table": {"alpha": 0.5, "gamma": 0.9, "bins": 6},
    "table-qlambda": {"alpha": 0.5, "gamma": 0.9, "bins": 6, "lam": 0.8, "obgd_kappa": 2.0, "round_mode": 1},
}
N_CELLS = 6


def corridor(agent, steps: int) -> list[int]:
    """Epsilon-greedy on the 6-cell corridor (reward 1 at the right end); returns every action."""
    actions, pos = [], 0
    agent.episode_start()
    for _ in range(steps):
        s = [pos + 0.5]
        a = agent.act(s, 0.2)
        pos = max(0, pos + (1 if a else -1))
        done = pos == N_CELLS - 1
        agent.observe(s, a, 1.0 if done else 0.0, [pos + 0.5], done)
        actions.append(a)
        if done:
            pos = 0
            agent.episode_start()
    return actions


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", help="PIL link; default: start QEMU on a free TCP port")
    ap.add_argument("--no-qemu", action="store_true")
    ap.add_argument("--steps", type=int, default=3000)
    a = ap.parse_args()
    qemu = None
    link = a.link
    if not a.no_qemu:
        subprocess.run(["make", "-s", "-C", str(ROOT / "m4" / "pil"), "qemu"], check=True, capture_output=True)
        port = free_port()
        qemu = subprocess.Popen(["qemu-system-arm", "-M", "netduinoplus2", "-nographic", "-monitor", "none",
                                 "-semihosting-config", "enable=on,target=native",
                                 "-serial", f"tcp:127.0.0.1:{port},server=on,wait=off,nodelay=on",
                                 "-kernel", str(ROOT / "build" / "pil" / "qemu" / "pil.elf")],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        link = link or f"tcp:127.0.0.1:{port}"
        time.sleep(0.5)
    report, ok = {}, True
    try:
        for name, L in LEARNERS.items():
            t0 = time.time()
            host = corridor(Agent(L, [0.0], [float(N_CELLS)], 2, seed=3), a.steps)
            t1 = time.time()
            chip_agent = PilAgent({**L, "pil": link}, [0.0], [float(N_CELLS)], 2, seed=3)
            chip = corridor(chip_agent, a.steps)
            t2 = time.time()
            stats = chip_agent.stats()
            chip_agent.close()
            match = host == chip
            ok &= match
            report[name] = {"match": match, "steps": a.steps, "host_s": round(t1 - t0, 2),
                            "chip_s": round(t2 - t1, 2), "board": stats,
                            "board_arena_bytes": chip_agent.mem_bytes}
            print(f"{name:14s} {'MATCH' if match else 'DIFFERENT':9s} steps {a.steps}  "
                  f"host {t1 - t0:5.1f}s  chip {t2 - t1:5.1f}s  arena {chip_agent.mem_bytes} B  "
                  f"stack {stats['stack_peak_bytes']} B")
    finally:
        if qemu:
            qemu.kill()
    out = ROOT / "results" / "m4" / "pil_check.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"link": "qemu" if qemu else link, "steps": a.steps, "learners": report}, indent=2))
    print("PIL check", "passed: on-chip learning reproduces the host bit for bit" if ok else "FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
