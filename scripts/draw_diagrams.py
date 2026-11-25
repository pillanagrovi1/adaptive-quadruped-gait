#!/usr/bin/env python3
"""Diagrams for docs/robot.md, drawn from the measured numbers (models/*.json).

    docs/img/leg_side.png   one leg in its own plane, CAD pose, every pivot and length labelled
    docs/img/top_view.png   hips, roll axes and feet seen from above (model frame, standing pose)
"""
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
G = json.loads((ROOT / "models" / "geometry_measured.json").read_text())
IMG = ROOT / "docs" / "img"


def leg_side():
    g = G["legs"]["FL"]
    P = {k: np.array(v) for k, v in g["plane_points_mm"].items()}
    fig, ax = plt.subplots(figsize=(7.5, 8), dpi=110)

    def seg(a, b, col, lw, label=None, off=(0, 0)):
        ax.plot(*np.c_[P[a], P[b]], color=col, lw=lw, solid_capstyle="round", zorder=2)
        if label:
            mid = (P[a] + P[b]) / 2 + np.array(off)
            ax.text(*mid, label, fontsize=9, ha="center", va="center",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85), zorder=5)

    seg("H", "K", "#e8e8e8", 14)
    seg("H", "K", "0.45", 2, f"femur {g['femur_mm']:.1f}", (12, 6))
    seg("K", "F", "#d84a3a", 9, f"tibia (knee to foot contact) {g['tibia_knee_to_foot_mm']:.1f}", (40, 0))
    seg("K", "T", "#d84a3a", 9, f"tibia lever {g['tibia_lever_mm']:.1f}", (-30, -4))
    seg("B2", "T", "0.1", 2, f"long rod {g['long_rod_mm']:.1f}", (-26, 0))
    seg("H", "B2", "#3a78c2", 4, f"bellcrank {g['bellcrank_long_mm']:.1f}", (-14, 10))
    seg("H", "B1", "#3a78c2", 4, f"{g['bellcrank_short_mm']:.1f}", (-6, 8))
    seg("A", "B1", "0.1", 2, f"short rod {g['short_rod_mm']:.1f}", (30, 14))
    seg("S", "A", "#8a5ac2", 4, f"knee-servo horn {g['horn_mm']:.1f}", (36, -6))
    seg("H", "S", "0.6", 1, f"{g['knee_servo_offset_mm']:.1f}", (12, -2))
    ax.add_patch(plt.Circle(P["Fc"], g["foot_radius_mm"], color="0.15", zorder=3))
    ax.annotate("H  hip pitch axis (femur servo);\nthe bellcrank also turns about H", xy=P["H"], xytext=(-105, 30),
                fontsize=9, arrowprops=dict(arrowstyle="->", color="0.3"), zorder=6)
    names = {"H": "", "S": "S  knee servo axis", "K": "K  knee (2 x 684Z bearings)",
             "A": "A", "B1": "B1", "B2": "B2", "T": "T", "F": f"F  foot contact (tip radius {g['foot_radius_mm']:.1f})"}
    offs = {"H": (6, 8), "S": (6, -10), "K": (-8, -14), "A": (5, 3), "B1": (4, 4), "B2": (-12, 4), "T": (-12, -6), "F": (6, -8)}
    for k, t in names.items():
        ax.plot(*P[k], "o", color="k", ms=4, zorder=4)
        ax.text(P[k][0] + offs[k][0], P[k][1] + offs[k][1], t, fontsize=9, zorder=6)
    ax.axhline(P["F"][1], color="0.7", lw=1, ls="--")
    ax.annotate("", xy=(55, P["F"][1]), xytext=(55, 0), arrowprops=dict(arrowstyle="<->", color="0.4"))
    ax.text(58, P["F"][1] / 2, f"{-P['F'][1]:.1f}\n(hip axis to\nfoot, CAD pose)", fontsize=8, color="0.3")
    ax.set_aspect("equal")
    ax.set_xlim(-115, 110)
    ax.set_ylim(-175, 40)
    ax.set_xlabel("forward (mm)  ->  front of the robot")
    ax.set_ylabel("up (mm)")
    ax.set_title("Front-left leg, side view in the leg plane (CAD pose). All lengths in mm,\n"
                 "pivot to pivot, measured from the STEP. Both push-rod loops are parallelograms.", fontsize=10)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(IMG / "leg_side.png")


def top_view():
    from quad.model import LEGS, load
    from quad.stand import place, stand_pose
    m, d = load()
    place(m, d, stand_pose(m))
    b = G["body"]
    fig, ax = plt.subplots(figsize=(8, 5.5), dpi=110)
    L, W = b["length_z_mm"], b["width_x_mm"]
    ax.add_patch(plt.Rectangle((-L / 2, -W / 2), L, W, fc="#dfe3e8", ec="0.4", zorder=1))
    ax.text(0, 0, f"body {L:.1f} x {W:.1f} mm\n(height {b['height_y_mm']:.1f} mm)", ha="center", va="center", fontsize=9)
    for leg in LEGS:
        hip = d.xpos[m.body(f"{leg}_hip").id] * 1000
        H = d.xpos[m.body(f"{leg}_femur").id] * 1000
        F = d.geom_xpos[m.geom(f"{leg}_foot").id] * 1000
        ax.plot([hip[0] - 40, hip[0] + 40], [hip[1], hip[1]], color="#8a5ac2", lw=1, ls="--")
        ax.plot(*hip[:2], "s", color="#8a5ac2", ms=6, zorder=3)
        ax.plot([hip[0], H[0]], [hip[1], H[1]], color="0.2", lw=2, zorder=3)
        ax.plot([H[0], F[0]], [H[1], F[1]], color="#d84a3a", lw=1, ls=":", zorder=3)
        ax.plot(*F[:2], "o", color="k", ms=6, zorder=4)
        ax.text(hip[0], hip[1] + (-14 if hip[1] > 0 else 9), f"{leg} hip ({hip[0]:.1f}, {hip[1]:.1f})", fontsize=8,
                ha="center")
        ax.text(F[0] + 4, F[1] + (6 if F[1] > 0 else -10), f"foot y {F[1]:.1f}", fontsize=8)
    g = G["legs"]["FL"]
    ax.annotate(f"hip roll axis to femur servo face {g['roll_to_femur_servo_face_lateral_mm']:.2f}\n"
                f"+ to foot plane {g['lateral_from_femur_servo_face_mm']['F']:.1f}", xy=(88.65, 60), xytext=(110, 105),
                fontsize=8, arrowprops=dict(arrowstyle="->", color="0.4"))
    ax.annotate("", xy=(88.65, -125), xytext=(-108.65, -125), arrowprops=dict(arrowstyle="<->", color="0.4"))
    ax.text(-10, -137, "hip to hip, front to rear: 197.3", fontsize=8, ha="center")
    ax.set_aspect("equal")
    ax.set_xlim(-190, 210)
    ax.set_ylim(-150, 130)
    ax.set_xlabel("x forward (mm)")
    ax.set_ylabel("y left (mm)")
    ax.set_title("Top view, standing pose. Squares: hip roll axes (dashed, along x). Dots: feet.", fontsize=10)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(IMG / "top_view.png")


if __name__ == "__main__":
    IMG.mkdir(parents=True, exist_ok=True)
    leg_side()
    top_view()
    print("wrote docs/img/leg_side.png and docs/img/top_view.png")
