#!/usr/bin/env python3
"""Stage 1.1 gate: servo step test per profile, delay test, stand test (default profile, every delay).

Writes results/stage1_1.json (the numbers in docs/results.md) and docs/img/stage1_1_step.png.
Needs matplotlib for the plot (requirements-reference.txt); the numbers do not.

    python3 scripts/stage1_1_report.py
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from quad.model import config  # noqa: E402
from quad.stand import stand_test  # noqa: E402
from quad.step_test import step_response  # noqa: E402

PROFILES = ("sts3215_7v4", "sts3215_12v", "spt5435_guess")
OUT = ROOT / "results" / "stage1_1.json"
PNG = ROOT / "docs" / "img" / "stage1_1_step.png"
COLORS = ("#257de0", "#f47230", "#00b894")    # dataviz reference palette, categorical slots 1-3
STYLES = ("-", "--", ":")


def strip(r):
    return {k: v for k, v in r.items() if k not in ("t", "q", "v")}


def main():
    cfg = config()
    sweep = cfg["delay_ms"]["sweep"]
    tick_s = 1.0 / cfg["control_hz"]["value"]
    dt = cfg["timestep_s"]["value"]
    steps, delays, traces = {}, {}, {}
    for p in PROFILES:
        r = step_response(p)
        traces[p] = r
        r = strip(r)
        r["speed_error_frac"] = r["peak_speed_rad_s"] / r["no_load_speed_rad_s"] - 1
        r["gate_pass"] = abs(r["speed_error_frac"]) <= 0.15
        steps[p] = r
        delays[p] = []
        for dl in sweep:
            s = strip(step_response(p, dl))
            s["start_shift_s"] = s["t_start_s"] - r["t_start_s"]
            s["gate_pass"] = abs(s["start_shift_s"] - dl / 1000) <= dt + tick_s
            delays[p].append(s)
    stands = [stand_test(dl) for dl in sweep]
    for s in stands:
        s["gate_pass"] = (max(s["torque_max_frac_stall"].values()) < 0.5 and not s["self_collisions"]
                          and not s["non_foot_floor_contacts"])
        s["above_half_rated"] = [k for k, v in s["torque_max_frac_rated"].items() if v > 0.5]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({"step": steps, "delay": delays, "stand": stands}, indent=1))

    for p, r in steps.items():
        print(f"{p:14s} peak {r['peak_speed_rad_s']:.3f} rad/s vs no-load {r['no_load_speed_rad_s']:.3f} "
              f"({r['speed_error_frac']:+.1%}), model top {r['model_top_speed_rad_s']:.3f}, t95 {r['t95_s'] * 1000:.0f} ms, overshoot {r['overshoot_deg']:.1f} deg, "
              f"pass {r['gate_pass']}")
        print("   delay shifts (ms):", ", ".join(f"{s['delay_ms']:.0f}->{s['start_shift_s'] * 1000:.0f}"
                                          f"{'' if s['gate_pass'] else ' FAIL'}" for s in delays[p]))
    for s in stands:
        fr, rr = s["torque_max_frac_stall"], s["torque_max_frac_rated"]
        print(f"stand {s['delay_ms']:.0f} ms: max {max(fr.values()):.3f} of stall, {max(rr.values()):.3f} of rated "
              f"({max(rr, key=rr.get)}), above half rated: {len(s['above_half_rated'])}, settle {s['settle_drop_mm']:.2f} mm, "
              f"drift {s['height_drift_1s_to_end_mm']:+.3f} mm, tilt {s['tilt_max_deg']:.3f} deg, pass {s['gate_pass']}")
    print("wrote", OUT.relative_to(ROOT))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed: no plot")
        return
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.8), dpi=130)
    for (p, r), c, ls in zip(traces.items(), COLORS, STYLES):
        a1.plot(r["t"] * 1000, [math.degrees(x) for x in r["q"]], color=c, ls=ls, lw=2, label=p)
        a2.plot(r["t"] * 1000, r["v"], color=c, ls=ls, lw=2, label=p)
        a2.axhline(r["no_load_speed_rad_s"], color=c, lw=1, ls=ls, alpha=0.5)
        a1.annotate(f"{p}  95 % at {steps[p]['t95_s'] * 1000:.0f} ms", (steps[p]["t95_s"] * 1000, 57),
                    xytext=(steps[p]["t95_s"] * 1000 + 15, 40 - 9 * PROFILES.index(p)), fontsize=8, color="#191919",
                    arrowprops={"arrowstyle": "-", "color": c, "lw": 1})
    a1.axhline(60, color="#666666", lw=0.8)
    a1.set(xlabel="time after the command (ms)", ylabel="servo angle (deg)", title="60 deg step, no load, no gravity", xlim=(0, 400))
    a2.set(xlabel="time after the command (ms)", ylabel="speed (rad/s)", title="speed (thin line = the profile's no-load spec)", xlim=(0, 400))
    for a in (a1, a2):
        a.grid(color="#e5e5e5", lw=0.6)
        a.spines[["top", "right"]].set_visible(False)
    a2.legend(frameon=False, fontsize=8, loc="upper right")
    fig.tight_layout()
    PNG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(PNG)
    print("wrote", PNG.relative_to(ROOT))


if __name__ == "__main__":
    main()
