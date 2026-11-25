#!/usr/bin/env python3
"""Knee push-rod linkage: tibia angle as a function of the knee-servo angle.

Uses only models/geometry_measured.json (no CAD needed). For each leg it solves the two
four-bar loops in the leg plane (points named as in docs/robot.md):

    loop 1 (in the hip module):  S -horn- A -28 mm rod- B1 -bellcrank- H      (S, H fixed)
    loop 2 (on the femur):       H -bellcrank- B2 -100 mm rod- T -tibia- K    (H, K on the femur)

The bellcrank turns about the hip pitch axis H; B1 and B2 are its two arms.
The servo angle sweeps its 180 deg travel, centred on the CAD pose (an assumption).
Writes models/knee_linkage.json and docs/img/knee_linkage.png.

    python3 scripts/knee_linkage.py
"""
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
GEOM = json.loads((ROOT / "models" / "geometry_measured.json").read_text())
CFG = json.loads((ROOT / "configs" / "robot.json").read_text())
OUT = ROOT / "models" / "knee_linkage.json"


def circle_meet(c0, r0, c1, r1, near):
    """Intersection of two circles nearest to `near` (keeps the branch continuous). None if apart."""
    d = np.linalg.norm(c1 - c0)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    m = c0 + a * (c1 - c0) / d
    perp = np.array([-(c1 - c0)[1], (c1 - c0)[0]]) / d
    p, q = m + h * perp, m - h * perp
    return p if np.linalg.norm(p - near) <= np.linalg.norm(q - near) else q


def angle(v):
    return math.atan2(v[1], v[0])


def rot(v, t):
    c, s = math.cos(t), math.sin(t)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def pitch(v):
    """Pitch of a leg-plane direction (fwd, up): 0 = straight down, + = foot backward (model convention)."""
    return math.atan2(-v[0], -v[1])


def transmission(u, v):
    """Angle between rod and the link it drives, folded to [0, 90] deg; 0 = dead point."""
    c = abs(np.dot(u, v)) / (np.linalg.norm(u) * np.linalg.norm(v))
    return math.degrees(math.acos(min(1.0, c)))


def solve(g, d_servo, d_femur=0.0):
    """Tibia pitch (rad) and the two loops' transmission angles for a servo turn of d_servo
    (+ = tibia foot backward) and a femur turn of d_femur from the CAD pose. None if it cannot close."""
    P = {k: np.array(v) for k, v in g["plane_points_mm"].items()}
    H, S = P["H"], P["S"]
    # femur rotation (about H) moves K and T's circle centre; pitch + = clockwise in (fwd, up)
    K = rot(P["K"] - H, -d_femur) + H
    A = rot(P["A"] - S, -d_servo) + S
    B1 = circle_meet(H, g["bellcrank_short_mm"], A, g["short_rod_mm"], rot(P["B1"] - H, -d_servo) + H)
    if B1 is None:
        return None
    turn = angle(B1 - H) - angle(P["B1"] - H)
    B2 = rot(P["B2"] - H, turn) + H
    T = circle_meet(K, g["tibia_lever_mm"], B2, g["long_rod_mm"], rot(P["T"] - P["K"], turn) + K)
    if T is None:
        return None
    lever = T - K
    foot = rot(lever, math.radians(g["tibia_lever_to_foot_deg"]))
    return {"tibia_pitch": pitch(foot), "t1": transmission(A - S, B1 - A), "t1b": transmission(B1 - H, B1 - A),
            "t2": transmission(B2 - H, T - B2), "t2b": transmission(T - K, T - B2)}


def analyse(g):
    half = math.radians(CFG["servo"]["travel_deg"]["value"] / 2)
    margin = CFG["joint_limit_margin_deg"]["value"]
    cad = solve(g, 0.0)["tibia_pitch"]
    rows = []
    for d in np.linspace(-half, half, 361):
        r = solve(g, d)
        if r is None:
            rows.append({"servo_deg": math.degrees(d), "ok": False})
            continue
        t_min = min(r["t1"], r["t1b"], r["t2"], r["t2b"])
        rows.append({"servo_deg": math.degrees(d), "ok": t_min >= margin,
                     "tibia_deg": math.degrees(r["tibia_pitch"] - cad), "transmission_min_deg": t_min})
    ok = [r for r in rows if r["ok"]]
    # contiguous usable range around the CAD pose
    i0 = next(i for i, r in enumerate(rows) if abs(r["servo_deg"]) < 1e-9)
    lo = hi = i0
    while lo > 0 and rows[lo - 1]["ok"]:
        lo -= 1
    while hi < len(rows) - 1 and rows[hi + 1]["ok"]:
        hi += 1
    use = rows[lo:hi + 1]
    dev = max(abs(r["tibia_deg"] - r["servo_deg"]) for r in use)
    s = np.array([r["servo_deg"] for r in use])
    t = np.array([r["tibia_deg"] for r in use])
    slope = np.gradient(t, s)
    # does the femur angle change the tibia's absolute angle? (should not: loop 2 is a parallelogram)
    femur_effect = 0.0
    for df in np.radians([-30, -15, 15, 30]):
        for d in np.radians([-40, 0, 40]):
            r0, r1 = solve(g, d), solve(g, d, df)
            if r0 and r1:
                femur_effect = max(femur_effect, abs(math.degrees(r1["tibia_pitch"] - r0["tibia_pitch"])))
    return {"cad_tibia_pitch_deg": math.degrees(cad), "usable_servo_deg": [use[0]["servo_deg"], use[-1]["servo_deg"]],
            "usable_tibia_deg": [use[0]["tibia_deg"] + math.degrees(cad), use[-1]["tibia_deg"] + math.degrees(cad)],
            "max_dev_from_1to1_deg": dev, "slope_min": float(slope.min()), "slope_max": float(slope.max()),
            "femur_effect_on_tibia_deg": femur_effect, "n_rows_usable": len(use), "sweep": rows}


def plot(results):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.4, 4.2), dpi=120)
    for leg, r in results.items():
        u = [x for x in r["sweep"] if x.get("tibia_deg") is not None]
        ax.plot([x["servo_deg"] for x in u], [x["tibia_deg"] for x in u], lw=1.5, label=leg)
    lo, hi = results["FL"]["usable_servo_deg"]
    ax.axvspan(lo, hi, color="0.92", zorder=0, label="usable (FL)")
    ax.plot([-90, 90], [-90, 90], "k--", lw=0.8, label="1:1")
    ax.set_xlabel("knee servo turn from the CAD pose (deg)")
    ax.set_ylabel("tibia turn from the CAD pose (deg)")
    ax.set_title("Knee push-rods: tibia angle vs knee-servo angle")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    (ROOT / "docs" / "img").mkdir(parents=True, exist_ok=True)
    fig.savefig(ROOT / "docs" / "img" / "knee_linkage.png")


def main():
    res = {leg: analyse(g) for leg, g in GEOM["legs"].items()}
    OUT.write_text(json.dumps(res, indent=1))
    for leg, r in res.items():
        print(f"{leg}: usable servo {r['usable_servo_deg'][0]:+.1f}..{r['usable_servo_deg'][1]:+.1f} deg, "
              f"tibia pitch {r['usable_tibia_deg'][0]:+.1f}..{r['usable_tibia_deg'][1]:+.1f} deg, "
              f"max |tibia - servo| {r['max_dev_from_1to1_deg']:.3f} deg, slope {r['slope_min']:.4f}..{r['slope_max']:.4f}, "
              f"femur changes tibia by <= {r['femur_effect_on_tibia_deg']:.4f} deg")
    plot(res)
    print("wrote", OUT.relative_to(ROOT), "and docs/img/knee_linkage.png")


if __name__ == "__main__":
    main()
