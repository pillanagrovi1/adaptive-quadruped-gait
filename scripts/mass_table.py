#!/usr/bin/env python3
"""Mass of every part, and mass / centre of mass / inertia of every model body.

Needs the reference design in ~/reference/quadruped/ (not in the repo). Writes
models/mass_measured.json, which scripts/make_model.py turns into the MJCF.

Rules (every density and purchased-part mass is in configs/robot.json with its source):
  printed parts     STL volume x density x infill factor (PLA; the foot is TPU)
  aluminium plates  DXF net area x thickness (from the drawing's note) x density
  servos, cells     fixed mass each
  everything else   STEP volume x an assumed density for its material
Each part's mass goes into the body it moves with; its inertia comes from the STEP solid,
scaled from the solid's volume to its mass (so a part is treated as uniform).

    python3 scripts/mass_table.py
"""
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
import dxf_area  # noqa: E402
import measure_reference as mr  # noqa: E402
from quad import stl  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CFG = json.loads((ROOT / "configs" / "robot.json").read_text())
OUT = ROOT / "models" / "mass_measured.json"
STL_DIR = mr.REF / "3dp parts_stl_"

# STEP part name -> STL file holding the same printed part (volumes checked equal within 0.5 %)
PRINTED = {
    "gövde1-V2": "gövde1-V2_1pc", "gövde2-V2": "gövde2-V2_1pc", "ön1-V2": "ön1-V2_1pcs",
    "ARKA1-V2-SWITCH": "ARKA1-V2-SWITCH_1pc", "femur v2": "femur v2_4pcs",
    "2servo holder2": "servo holder_L_2pcs", "Aynalama2servo holder2": "servo holder_R_2pcs",
    "2servo holder3": "servo holderB_L_2pcs", "Aynalama2servo holder3": "servo holderB_R_2pcs",
    "servo arm mod-v2": "servo arm mod-v2_L_2pcs", "Aynalamaservo arm mod-v2": "servo arm mod-v2_R_2pcs",
    "spacer-for100mm": "spacer-for100mm_4pcs",
    "tibia v4": "tibia v4_L-onepice_2pcs", "mirror-tibia v4": "tibia v4_R-onepice_2pcs",
}


def stl_volume_cm3(name):
    return stl.volume(stl.load(STL_DIR / f"{name}.STL")) / 1000.0


def vol_props(shape):
    """(volume mm^3, centre mm, inertia about the centre for unit density, mm^5)."""
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    c = g.CentreOfMass()
    m = g.MatrixOfInertia()
    I = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return g.Mass(), np.array([c.X(), c.Y(), c.Z()]), I


def corners(shape):
    """The 8 corners of a part's bounding box (CAD frame, mm)."""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    lo, hi = box.CornerMin(), box.CornerMax()
    L, Hh = (lo.X(), lo.Y(), lo.Z()), (hi.X(), hi.Y(), hi.Z())
    return np.array([[(L, Hh)[i][0], (L, Hh)[j][1], (L, Hh)[k][2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)])


def part_mass(p, vol_mm3, plates):
    """(mass in g, rule text) for one STEP part instance."""
    d, D = p["def"], CFG["density_g_cm3"]
    infill = CFG["printed_infill_factor"]["value"]
    if d in PRINTED:
        v = stl_volume_cm3(PRINTED[d])
        if "tibia" in d:   # the STEP tibia includes the TPU foot
            vf = stl_volume_cm3("tibia foot TPUflex_4pcs")
            m = v * D["PLA"]["value"] * infill + vf * D["TPU"]["value"] * infill
            return m, f"STL {v:.2f} cm3 PLA + foot {vf:.2f} cm3 TPU, x infill {infill}"
        return v * D["PLA"]["value"] * infill, f"STL {v:.2f} cm3 x PLA x infill {infill}"
    if d == "spt5435LV":
        return CFG["servo"]["mass_g"]["value"], "servo, fixed"
    if d == "3x18650":
        v = stl_volume_cm3("3x18650case_2pcs") + stl_volume_cm3("3x18650caseCap_2pcs")
        cell = CFG["cell_18650_mass_g"]["value"]
        return 3 * cell + v * D["PLA"]["value"] * infill, f"3 cells x {cell} g + case STL {v:.2f} cm3 PLA"
    if d in ("üst sac", "alt sac"):
        t = CFG["plate_thickness_mm"]["value"]
        # match the plate by length: top 297 mm, bottom 278 mm
        want = 297.1 if d == "üst sac" else 278.0
        pl = min(plates, key=lambda q: abs(max(q["size_mm"]) - want))
        return pl["net_mm2"] * t / 1000 * D["aluminium"]["value"], f"DXF {pl['net_mm2']:.0f} mm2 x {t} mm x Al"
    if d == "rodend 5mmPlastic":
        return vol_mm3 / 1000 * D["POM"]["value"], "STEP volume x POM"
    if d.startswith("Servo Mount - Arm Metal"):
        return vol_mm3 / 1000 * D["aluminium"]["value"], "STEP volume x Al"
    if d in ("servo arm 1", "servo arm round"):
        return vol_mm3 / 1000 * D["nylon"]["value"], "STEP volume x nylon"
    if d in ("684z bearing", "rodend 5mmBall") or "screw" in d:
        return vol_mm3 / 1000 * D["steel"]["value"], "STEP volume x steel"
    if d in CFG["electronics_density_g_cm3"]:
        e = CFG["electronics_density_g_cm3"][d]
        return vol_mm3 / 1000 * e["value"], f"STEP volume x {e['value']} g/cm3"
    raise KeyError(f"no mass rule for {d!r}")


def rods(leg_geom):
    """The two push-rods are not solids in the STEP: steel rods of the measured ball-centre length."""
    dia = CFG["push_rod_diameter_mm"]["value"]
    out = []
    for name, L in (("100 mm push-rod", leg_geom["long_rod_mm"]), ("28 mm push-rod", leg_geom["short_rod_mm"])):
        out.append((name, math.pi * (dia / 2) ** 2 * L / 1000 * CFG["density_g_cm3"]["steel"]["value"], L))
    return out


def Ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def frames(g):
    """Rotation (CAD vectors -> body axes) and origin (CAD, mm) of each body of one leg, at the
    model's zero pose: leg plane vertical, femur straight down, tibia straight down."""
    s = 1.0 if g["side"] == "left" else -1.0
    a = np.array(g["pitch_axis_dir"])
    fwd = np.array([0.0, 0.0, 1.0])
    up = np.cross(fwd, s * a)
    Rm = np.vstack([fwd, s * a, up])                 # CAD -> module axes (x fwd, y left, z up)
    hip = np.array(g["hip_world_mm"])
    H = hip + a * g["roll_to_femur_servo_face_lateral_mm"]
    pp = g["plane_points_mm"]
    lat = g["lateral_from_femur_servo_face_mm"]
    K = H + a * lat["K"] + fwd * pp["K"][0] + up * pp["K"][1]
    F = H + a * lat["F"] + fwd * pp["F"][0] + up * pp["F"][1]
    dK = Rm @ (K - H)
    th = math.atan2(-dK[0], -dK[2])                  # femur pitch in the CAD pose
    dF = Rm @ (F - K)
    ps = math.atan2(-dF[0], -dF[2])                  # tibia pitch (absolute) in the CAD pose
    return {"hip": (Rm, hip), "femur": (Ry(-th) @ Rm, H), "tibia": (Ry(-ps) @ Rm, K)}, th, ps


def leg_body(p, legname):
    """Which leg body a part of the leg sub-assembly moves with."""
    d = p["def"]
    if d in ("femur v2", "servo arm round", "servo arm mod-v2", "Aynalamaservo arm mod-v2", "spacer-for100mm"):
        return "femur"
    if "tibia" in d or d == "684z bearing":
        return "tibia"
    if d.startswith("rodend"):
        # rod ends: the 100 mm rod (with the spacer) moves with the femur, the 28 mm rod with the hip module
        return "femur" if p["path"][1] in p["_spacer_paths"] else "hip"
    return "hip"


def main():
    for f in (mr.STEP, dxf_area.DXF):
        if not f.exists():
            sys.exit(f"{f} not found: unzip the reference design to {mr.REF}")
    plates, _ = dxf_area.measure()
    parts = mr.load_parts()
    geom = json.loads(mr.OUT.read_text())
    legkey = {v: k for k, v in mr.LEGS.items()}
    rows = []
    bodies = defaultdict(lambda: {"m": 0.0, "mc": np.zeros(3), "I": np.zeros((3, 3)), "items": []})

    # per-leg frames; screws inside a leg go to the body of the nearest non-screw part
    F = {n: frames(g)[0] for n, g in geom["legs"].items()}
    props = [vol_props(p["shape"]) for p in parts]
    for key, n in mr.LEGS.items():
        sp = {p["path"][1] for p in parts if p["path"][:1] == [key] and p["def"] == "spacer-for100mm"}
        for p in parts:
            if p["path"][:1] == [key]:
                p["_spacer_paths"] = sp

    def add(body, frame, mass_g, centre, I_unit, vol, corners=None):
        R, o = frame
        c = R @ (centre - o)
        if corners is not None:
            q = (R @ (corners - o).T).T
            b0 = bodies[body]
            b0.setdefault("lo", np.full(3, np.inf))
            b0.setdefault("hi", np.full(3, -np.inf))
            b0["lo"] = np.minimum(b0["lo"], q.min(0))
            b0["hi"] = np.maximum(b0["hi"], q.max(0))
        I = R @ (I_unit * (mass_g / vol if vol > 0 else 0.0)) @ R.T     # g mm^2 about the part's centre
        b = bodies[body]
        b["m"] += mass_g
        b["mc"] += mass_g * c
        b["items"].append((mass_g, c, I))

    torso_frame = (np.array([[0, 0, 1.0], [1.0, 0, 0], [0, 1.0, 0]]), np.zeros(3))   # CAD -> MJCF (x fwd, y left, z up)
    for p, (vol, c, Iu) in zip(parts, props):
        m, rule = part_mass(p, vol, plates)
        leg = mr.LEGS.get(p["path"][0]) if p["path"] else None
        if leg is None:
            body = "torso"
            frame = torso_frame
        else:
            if "screw" in p["def"]:
                cand = [(np.linalg.norm(c2 - c), q) for q, (_, c2, _) in zip(parts, props)
                        if q["path"][:1] == p["path"][:1] and "screw" not in q["def"]]
                body = leg_body(min(cand, key=lambda t: t[0])[1], leg)
            else:
                body = leg_body(p, leg)
            frame = F[leg][body]
            body = f"{leg}_{body}"
        add(body, frame, m, c, Iu, vol, corners(p["shape"]) if "screw" not in p["def"] else None)
        rows.append({"part": p["def"], "body": body, "mass_g": round(m, 3), "rule": rule})

    for leg, g in geom["legs"].items():
        fr = F[leg]
        pp = g["plane_points_mm"]
        for name, m, L in rods(g):
            body = "femur" if name.startswith("100") else "hip"
            # rod centre and direction from its ball centres (in the leg plane); thin-rod inertia
            ends = ("B2", "T") if body == "femur" else ("A", "B1")
            s = 1.0 if g["side"] == "left" else -1.0
            a = np.array(g["pitch_axis_dir"])
            up = np.cross(np.array([0, 0, 1.0]), s * a)
            Hc = np.array(g["hip_world_mm"]) + a * g["roll_to_femur_servo_face_lateral_mm"]
            lat = g["lateral_from_femur_servo_face_mm"]
            P = [Hc + a * lat[e] + np.array([0, 0, 1.0]) * pp[e][0] + up * pp[e][1] for e in ends]
            c = (P[0] + P[1]) / 2
            u = (P[1] - P[0]) / np.linalg.norm(P[1] - P[0])
            Iu = (np.eye(3) - np.outer(u, u)) * L * L / 12.0          # per unit mass
            add(f"{leg}_{body}", fr[body], m, c, Iu, 1.0)
            rows.append({"part": name, "body": f"{leg}_{body}", "mass_g": round(m, 3),
                         "rule": f"steel rod {CFG['push_rod_diameter_mm']['value']} mm x {L:.1f} mm (not in the STEP)"})

    out = {"total_g": round(sum(r["mass_g"] for r in rows), 2), "bodies": {}, "parts": rows}
    for name, b in sorted(bodies.items()):
        com = b["mc"] / b["m"]
        I = np.zeros((3, 3))
        for m, c, Ip in b["items"]:
            r = c - com
            I += Ip + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        out["bodies"][name] = {"mass_g": round(b["m"], 3), "com_mm": com.round(3).tolist(),
                               "aabb_mm": [b["lo"].round(2).tolist(), b["hi"].round(2).tolist()],
                               "inertia_g_mm2": [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]}
    out["cad_pose_deg"] = {leg: {"hip_pitch": math.degrees(frames(g)[1]),
                                 "tibia_abs": math.degrees(frames(g)[2]),
                                 "knee": math.degrees(frames(g)[2] - frames(g)[1]),
                                 "hip_roll": g["roll_angle_in_cad_deg"]} for leg, g in geom["legs"].items()}
    OUT.write_text(json.dumps(out, indent=1, ensure_ascii=False))
    print(f"total {out['total_g']:.1f} g")
    for n, b in out["bodies"].items():
        print(f"  {n:10s} {b['mass_g']:8.1f} g  com {b['com_mm']}")
    print("CAD pose:", {k: {a: round(v, 1) for a, v in d.items()} for k, d in out["cad_pose_deg"].items()})
    print("wrote", OUT.relative_to(ROOT))


if __name__ == "__main__":
    main()
