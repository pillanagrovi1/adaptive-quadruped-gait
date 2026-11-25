#!/usr/bin/env python3
"""Measure the reference robot from its STEP assembly (not in the repo).

Reads ~/reference/quadruped/Full assembly_step_/quadrupedKNGmain assy.STEP and writes
models/geometry_measured.json: joint axes, link lengths, foot, body size and the knee push-rod
linkage, for every leg. docs/robot.md quotes these numbers; the MJCF model is built from them.

How each number is found (see docs/robot.md for the picture):
  servo output axis      the servo's spline screw hole, a cylinder of radius 1.25 mm
  hip roll axis          the output axis of the body servo at that corner
  hip pitch axis         the output axis of the leg servo that carries the round horn (femur servo)
  knee axis              the axis of the two 684Z bearings in the tibia (their midpoint)
  push-rod joints        centres of the 5 mm ball joints ("rodend 5mmBall")
  foot                   the tibia's lowest tip; radius from a circle fitted to the tip's profile
Lengths are in the leg plane (perpendicular to the hip pitch axis) unless named "lateral".

    python3 scripts/measure_reference.py
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.GProp import GProp_GProps
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

sys.path.insert(0, str(Path(__file__).resolve().parent))
from step_tree import STEP, REF, TDF_Label, TDF_LabelSequence, name_of  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "models" / "geometry_measured.json"
LEGS = {"NAUO6": "FL", "NAUO7": "FR", "NAUO8": "RL", "NAUO9": "RR"}   # +z is front ("ön"), +x is left


def load_parts():
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    r = STEPCAFControl_Reader()
    r.SetNameMode(True)
    r.ReadFile(str(STEP))
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    parts = []

    def walk(label, loc, path):
        ref = label
        if st.IsReference_s(label):
            ref = TDF_Label()
            st.GetReferredShape_s(label, ref)
        here = loc * st.GetLocation_s(label) if st.IsReference_s(label) else loc
        nm = name_of(label) if st.IsReference_s(label) else name_of(ref)
        if st.IsAssembly_s(ref):
            k = TDF_LabelSequence()
            st.GetComponents_s(ref, k)
            for i in range(1, k.Length() + 1):
                walk(k.Value(i), here, path + [nm])
            return
        parts.append({"def": name_of(ref), "path": path[1:], "shape": st.GetShape_s(ref).Moved(here)})

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    return parts


def cylinders(shape, rmin, rmax):
    """(radius, unit axis dir, point on the axis) for every cylindrical face with rmin <= r <= rmax."""
    out = []
    e = TopExp_Explorer(shape, TopAbs_FACE)
    while e.More():
        f = TopoDS.Face(e.Current())
        s = BRepAdaptor_Surface(f)
        if s.GetType() == GeomAbs_Cylinder and rmin <= s.Cylinder().Radius() <= rmax:
            c = s.Cylinder()
            ax = c.Axis()
            p = np.array([ax.Location().X(), ax.Location().Y(), ax.Location().Z()])
            d = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
            g = GProp_GProps()
            BRepGProp.SurfaceProperties_s(f, g)
            m = np.array([g.CentreOfMass().X(), g.CentreOfMass().Y(), g.CentreOfMass().Z()])
            out.append((c.Radius(), d, p + d * np.dot(m - p, d)))
        e.Next()
    return out


def centroid(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    c = g.CentreOfMass()
    return np.array([c.X(), c.Y(), c.Z()])


def mesh_points(shape, tol=0.2):
    BRepMesh_IncrementalMesh(shape, tol)
    pts = []
    e = TopExp_Explorer(shape, TopAbs_FACE)
    while e.More():
        f = TopoDS.Face(e.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            tr = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(tr)
                pts.append((p.X(), p.Y(), p.Z()))
        e.Next()
    return np.array(pts)


def servo_axis(shape):
    c = cylinders(shape, 1.2, 1.3)       # one hole, sometimes stored as two half-cylinder faces
    p0, d0 = c[0][2], c[0][1]
    for _, d, p in c[1:]:
        if abs(abs(np.dot(d, d0)) - 1) > 1e-6 or np.linalg.norm(np.cross(p - p0, d0)) > 1e-3:
            raise RuntimeError("spline hole faces are not on one axis")
    return p0, d0


def fit_circle(xy):
    """Least-squares circle (Kasa fit): returns centre (2,) and radius."""
    A = np.c_[2 * xy, np.ones(len(xy))]
    b = (xy ** 2).sum(1)
    cx, cy, k = np.linalg.lstsq(A, b, rcond=None)[0]
    return np.array([cx, cy]), math.sqrt(k + cx * cx + cy * cy)


def measure_leg(parts, key, body_servos):
    mine = [p for p in parts if p["path"] and p["path"][0] == key]
    servos = [p for p in mine if p["def"] == "spt5435LV"]
    horn_round = next(p for p in mine if p["def"] == "servo arm round")
    rc = cylinders(horn_round["shape"], 2.7, 2.8)[0]
    axes = [servo_axis(s["shape"]) for s in servos]
    # femur servo: its axis passes through the round horn's hub
    dist = [np.linalg.norm(np.cross(rc[2] - p, d)) for p, d in axes]
    i_f = int(np.argmin(dist))
    H, a = axes[i_f]
    S, _ = axes[1 - i_f]
    bearings = [p for p in mine if p["def"] == "684z bearing"]
    K = np.mean([cylinders(b["shape"], 4.4, 4.6)[0][2] for b in bearings], axis=0)
    # roll axis: the body servo nearest this leg
    rp, rd = min(body_servos, key=lambda ax: np.linalg.norm(np.cross(H - ax[0], ax[1])))
    # leg frame: a = pitch axis pointing outward; fwd = +z; up = the rest
    if np.sign(a[0]) != np.sign(H[0]):
        a = -a
    fwd = np.array([0.0, 0.0, 1.0])
    fwd = fwd - a * np.dot(fwd, a)
    fwd /= np.linalg.norm(fwd)
    up = np.cross(fwd, a) if a[0] > 0 else np.cross(a, fwd)
    if up[1] < 0:
        up = -up

    def plane(P):  # (lateral, forward, up) relative to H
        v = P - H
        return np.array([np.dot(v, a), np.dot(v, fwd), np.dot(v, up)])

    # push rods: the sub-assembly with the spacer is the 100 mm rod
    balls = [p for p in mine if p["def"] == "rodend 5mmBall"]
    rods = {}
    for b in balls:
        rods.setdefault(tuple(b["path"][:2]), []).append(centroid(b["shape"]))
    spacer_paths = {tuple(p["path"][:2]) for p in mine if p["def"] == "spacer-for100mm"}
    long_rod = next(v for k, v in rods.items() if k in spacer_paths)
    short_rod = next(v for k, v in rods.items() if k not in spacer_paths)
    # ends: the short rod's end nearer the knee servo is on its horn; the long rod's lower end is on the tibia
    A, B1 = sorted(short_rod, key=lambda P: np.linalg.norm(plane(P)[1:] - plane(S)[1:]))
    T, B2 = sorted(long_rod, key=lambda P: plane(P)[2])
    # foot: lowest tibia point along the leg's own down direction at the CAD pose
    tibia = next(p for p in mine if "tibia" in p["def"])
    pts = np.array([plane(P) for P in mesh_points(tibia["shape"])])
    k2 = plane(K)[1:]
    r = np.linalg.norm(pts[:, 1:] - k2, axis=1)
    tip = pts[np.argmax(r)]
    near = pts[np.linalg.norm(pts[:, 1:] - tip[1:], axis=1) < 6.0]
    ctr, rad = fit_circle(near[:, 1:])
    # the foot's contact point: on the fitted circle, in the direction away from the knee
    dirk = (ctr - k2) / np.linalg.norm(ctr - k2)
    F2 = ctr + dirk * rad

    P2 = {n: plane(P)[1:] for n, P in {"H": H, "S": S, "K": K, "A": A, "B1": B1, "B2": B2, "T": T}.items()}
    P2["F"] = F2
    P2["Fc"] = ctr
    lat = {n: float(plane(P)[0]) for n, P in {"S": S, "K": K, "T": T, "A": A, "B1": B1, "B2": B2}.items()}
    lat["F"] = float(np.mean(near[:, 0]))
    roll_pt = rp + rd * np.dot(H - rp, rd)            # point of the roll axis nearest the pitch axis
    L = lambda u, v: float(np.linalg.norm(P2[u] - P2[v]))  # noqa: E731
    ang = lambda u, c, v: float(math.degrees(math.atan2(*(lambda p, q: (p[0] * q[1] - p[1] * q[0], p @ q))(P2[u] - P2[c], P2[v] - P2[c]))))  # noqa: E731
    roll_cad = math.degrees(math.atan2(-up[0] * np.sign(H[0]), up[1]))
    return {
        "side": "left" if H[0] > 0 else "right", "end": "front" if H[2] > 0 else "rear",
        "hip_world_mm": roll_pt.round(2).tolist(),
        "roll_axis_dir": rd.round(4).tolist(),
        "pitch_axis_dir": a.round(4).tolist(),
        "roll_pitch_perp_offset_mm": float(abs(np.dot(H - rp, np.cross(rd, a)) / np.linalg.norm(np.cross(rd, a)))),
        "roll_to_femur_servo_face_lateral_mm": float(np.linalg.norm(H - roll_pt)),
        "lateral_from_femur_servo_face_mm": lat,
        "roll_angle_in_cad_deg": roll_cad,
        "plane_points_mm": {k: v.round(3).tolist() for k, v in P2.items()},
        "femur_mm": L("H", "K"),
        "tibia_knee_to_foot_mm": L("K", "F"),
        "foot_radius_mm": rad,
        "knee_servo_offset_mm": L("H", "S"),
        "horn_mm": L("S", "A"), "short_rod_mm": L("A", "B1"),
        "bellcrank_short_mm": L("H", "B1"), "bellcrank_long_mm": L("H", "B2"),
        "bellcrank_angle_deg": ang("B1", "H", "B2"),
        "long_rod_mm": L("B2", "T"), "tibia_lever_mm": L("K", "T"),
        "tibia_lever_to_foot_deg": ang("T", "K", "F"),
        "femur_cad_deg": math.degrees(math.atan2(*(P2["K"] - P2["H"])[::-1])),
    }


def body(parts):
    names = ("gövde1-V2", "gövde2-V2", "ön1-V2", "ARKA1-V2-SWITCH", "üst sac", "alt sac")
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    for p in parts:
        if p["def"] in names:
            BRepBndLib.Add_s(p["shape"], box)
    lo, hi = box.CornerMin(), box.CornerMax()
    return {"length_z_mm": hi.Z() - lo.Z(), "width_x_mm": hi.X() - lo.X(), "height_y_mm": hi.Y() - lo.Y(),
            "box_min": [lo.X(), lo.Y(), lo.Z()], "box_max": [hi.X(), hi.Y(), hi.Z()]}


def main():
    if not STEP.exists():
        sys.exit(f"{STEP} not found: unzip the reference design to {REF}")
    parts = load_parts()
    body_servos = [servo_axis(p["shape"]) for p in parts if p["def"] == "spt5435LV" and not p["path"]]
    imu = next(p for p in parts if p["def"] == "Orange Pi Zero")
    out = {"source": "quadrupedKNGmain assy.STEP (frame: x left, y up, z forward; mm)",
           "body": body(parts),
           "board_centroid_mm": centroid(imu["shape"]).round(2).tolist(),
           "legs": {}}
    for key, name in LEGS.items():
        out["legs"][name] = measure_leg(parts, key, body_servos)
    OUT.write_text(json.dumps(out, indent=1))
    for n, g in out["legs"].items():
        print(f"{n}: femur {g['femur_mm']:.2f}  tibia {g['tibia_knee_to_foot_mm']:.2f}  foot r {g['foot_radius_mm']:.2f}  "
              f"horn {g['horn_mm']:.2f} rod {g['short_rod_mm']:.2f} crank {g['bellcrank_short_mm']:.2f}/{g['bellcrank_long_mm']:.2f} "
              f"@{g['bellcrank_angle_deg']:.1f}  long rod {g['long_rod_mm']:.2f}  tibia lever {g['tibia_lever_mm']:.2f}  "
              f"S {g['knee_servo_offset_mm']:.2f}  roll {g['roll_angle_in_cad_deg']:.1f}  hip {g['hip_world_mm']}")
    print("body", {k: round(v, 1) for k, v in out["body"].items() if k.endswith("mm")})
    print("wrote", OUT.relative_to(ROOT))


if __name__ == "__main__":
    main()
