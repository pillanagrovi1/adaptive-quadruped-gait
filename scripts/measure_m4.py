#!/usr/bin/env python3
"""Measure Q-table configurations on a Cortex-M4 in QEMU.

For each configuration in CONFIGS it builds the firmware (m4/) with that exact tr_cfg_t,
runs it under QEMU (netduinoplus2, Cortex-M4) and records:

  agent_bytes, arena_bytes, stack_learner   -> peak_bytes = sum of the three
  flash text/data/bss                       -> from arm-none-eabi-size
  insns_learn_step                          -> instructions per act()+observe()
  insns_greedy_step                         -> instructions per greedy action
  fits                                      -> did it link into 128 KB SRAM?

Instruction counts come from differencing two runs (N1 and N2 steps) so that
startup and init cost cancel: (I(N2) - I(N1)) / (N2 - N1).
They include the benchmark's own synthetic-data generation (a few dozen
instructions per step); see m4/bench_main.c.

These are QEMU instruction counts, NOT cycles and NOT time on an STM32F446RE.

    python3 scripts/measure_m4.py smoke --check     # what CI runs
    python3 scripts/measure_m4.py smoke --only corridor_table
Output: results/m4/<set>.csv (one row per configuration)
"""
import argparse
import csv
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from tinyrl import cbind  # noqa: E402

PLUGIN = ROOT / "tools" / "qemu_plugin" / "libinsn_count.so"
OUT = ROOT / "results" / "m4"
N1, N2 = 100, 300
QEMU = ["qemu-system-arm", "-M", "netduinoplus2", "-nographic", "-monitor", "none",
        "-semihosting-config", "enable=on,target=native"]
SRAM = 128 * 1024

# Named sets of configurations: run_id -> (learner, obs_lo, obs_hi, n_actions).
# "smoke" is a pipeline check: the corridor table, with and without traces, and a
# table the size Stage 4 may need (4 observations x 6 bins, 7 actions; a guess, not a design).
CONFIGS = {
    "smoke": {
        "corridor_table": ({"alpha": 0.5, "gamma": 0.9, "bins": 6}, [0.0], [6.0], 2),
        "corridor_qlambda": ({"alpha": 0.5, "gamma": 0.9, "bins": 6, "lam": 0.8}, [0.0], [6.0], 2),
        "table_4d_6bins_7a": ({"alpha": 0.1, "gamma": 0.95, "bins": 6, "round_mode": 1},
                              [-1.0] * 4, [1.0] * 4, 7),
    },
}


def c_initializer(cfg: cbind.Cfg) -> str:
    d = cbind.cfg_dict(cfg)
    rename = {"lam": "lambda"}            # `lambda` is a Python keyword
    parts = []
    for k, v in d.items():
        k = rename.get(k, k)
        parts.append(f".{k} = {{{', '.join(str(x) for x in v)}}}" if isinstance(v, list) else f".{k} = {v}")
    return "{ " + ", ".join(parts) + " }"


def build(cfg, arena_bytes, steps, mode, outdir: Path):
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "bench_cfg.h").write_text(
        "#ifndef BENCH_CFG_H\n#define BENCH_CFG_H\n"
        f"#define BENCH_CFG {c_initializer(cfg)}\n"
        f"#define ARENA_BYTES {max(4, arena_bytes)}\n"
        f"#define BENCH_STEPS {steps}\n#define BENCH_MODE {mode}\n#endif\n")
    r = subprocess.run(["make", "-s", "-C", str(ROOT / "m4"), f"CFG_DIR={outdir}", f"OUT={outdir}"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        return None, r.stderr[-400:]
    m = re.search(r"^\s*(\d+)\s+(\d+)\s+(\d+)\s+\d+\s+\w+\s+\S*fw\.elf", r.stdout, re.M)
    size = dict(zip(("text", "data", "bss"), map(int, m.groups()))) if m else {}
    return size, ""


def run_qemu(elf: Path):
    cmd = QEMU + ["-plugin", str(PLUGIN), "-d", "plugin", "-kernel", str(elf)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    kv = dict(re.findall(r"^(\w+)=(\d+)$", r.stdout + r.stderr, re.M))
    return {k: int(v) for k, v in kv.items()}


def host_checksum(cfg_dir: Path):
    """Compile the same workload for x86-64 and return its checksum."""
    exe = cfg_dir / "bench_host"
    src = [str(ROOT / "c" / "tests" / "bench_host.c")] + [str(p) for p in sorted((ROOT / "c" / "src").glob("*.c"))]
    r = subprocess.run(["gcc", "-O2", "-std=c11", f"-I{cfg_dir}", f"-I{ROOT / 'c' / 'include'}",
                        f"-I{ROOT / 'c' / 'src'}", f"-I{ROOT / 'm4'}", *src, "-o", str(exe)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        return None
    out = subprocess.run([str(exe)], capture_output=True, text=True).stdout
    m = re.search(r"checksum=(\d+)", out)
    return int(m.group(1)) if m else None


def measure(run_id, spec, set_name):
    learner, lo, hi, n_actions = spec
    cfg = cbind.make_cfg(learner, lo, hi, n_actions)
    cid = cbind.cfg_id(cfg)
    arena = int(cbind.lib().tr_mem_bytes(cfg))
    base = ROOT / "build" / "m4" / cid
    row = {"set": set_name, "run_id": run_id, "cfg_id": cid, "arena_bytes": arena, "fits": 0}
    res = {}
    for mode in (0, 1):
        for n in (N1, N2):
            size, err = build(cfg, arena, n, mode, base / f"m{mode}_n{n}")
            if size is None:
                too_big = "static RAM too large" in err or "overflowed" in err
                row["error"] = "does not fit in SRAM (link failed)" if too_big else "build failed: " + err.strip()[-160:]
                row["peak_bytes_estimate"] = arena + 300  # rough, for plotting only
                return row
            res[(mode, n)] = (size, run_qemu(base / f"m{mode}_n{n}" / "fw.elf"))
    size, kv = res[(0, N2)]
    if kv.get("init_ok") != 1:
        row["error"] = "init failed on target"
        return row
    row.update(fits=1, agent_bytes=kv["agent_bytes"], stack_learner=kv["stack_learner"],
               stack_total=kv["stack_total"],
               peak_bytes=kv["agent_bytes"] + arena + kv["stack_learner"],
               flash_text=size.get("text"), ram_data=size.get("data"), ram_bss=size.get("bss"))
    for mode, name in ((0, "insns_learn_step"), (1, "insns_greedy_step")):
        i1 = res[(mode, N1)][1].get("insns")
        i2 = res[(mode, N2)][1].get("insns")
        row[name] = round((i2 - i1) / (N2 - N1), 1) if i1 and i2 else None
    row["fits"] = int(row["peak_bytes"] + (size.get("data") or 0) <= SRAM)
    hc = host_checksum(base / f"m0_n{N2}")
    row["host_match"] = int(hc is not None and hc == kv.get("checksum"))
    return row


FIELDS = ["set", "run_id", "cfg_id", "fits", "peak_bytes", "agent_bytes",
          "arena_bytes", "stack_learner", "stack_total", "insns_learn_step", "insns_greedy_step",
          "flash_text", "ram_data", "ram_bss", "host_match", "peak_bytes_estimate", "error"]


def broken(rows) -> list[str]:
    """Runs whose measurement cannot be used. Not fitting into the SRAM is a result, not a failure."""
    out = []
    for r in rows:
        err = str(r.get("error") or "")
        if err.startswith("does not fit"):
            continue
        if err or str(r.get("host_match")) != "1":
            out.append(r["run_id"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("set", choices=sorted(CONFIGS))
    ap.add_argument("--only")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if any config fails to build/run or host != M4 (used by CI)")
    a = ap.parse_args()
    if not PLUGIN.exists():
        sys.exit("build the QEMU plugin first: make -C tools/qemu_plugin")
    specs = {k: v for k, v in CONFIGS[a.set].items() if not a.only or k == a.only}
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{a.set}.csv"
    old = {}
    if path.exists():
        with open(path) as f:
            old = {r["run_id"]: r for r in csv.DictReader(f)}
    measured = []
    for k, (run_id, spec) in enumerate(specs.items(), 1):
        row = measure(run_id, spec, a.set)
        measured.append(row)
        old[run_id] = row
        print(f"[{k}/{len(specs)}] {run_id:40s} peak {row.get('peak_bytes', 'n/a')!s:>7} B  "
              f"learn {row.get('insns_learn_step', 'n/a')!s:>9} insn/step  host==m4 {row.get("host_match", "n/a")}  {row.get('error', '')}", flush=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        for r in old.values():
            w.writerow(r)
    print("wrote", path.relative_to(ROOT))
    unusable = broken(measured)
    if unusable:
        sys.exit(f"MEASUREMENT FAILED (build, run or host != M4) for: {', '.join(unusable)}")
    if a.check:
        bad = [r["run_id"] for r in old.values()
               if str(r.get("host_match")) != "1" or r.get("error")]
        if bad:
            sys.exit(f"CHECK FAILED for: {', '.join(bad)}")
        print("check passed: every config ran on the M4 and matched the host bit for bit")


if __name__ == "__main__":
    main()
