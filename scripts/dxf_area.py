#!/usr/bin/env python3
"""Area of the laser-cut plates in ~/reference/quadruped/Lasercut_dxf_/Lasercut parts.dxf (not in the repo).

A small plain-text DXF reader (no extra dependency): LINE and ARC entities are joined into closed
outlines, CIRCLE entities are holes. Plate area = outline area - holes inside it.

    python3 scripts/dxf_area.py
"""
import math
import sys
from pathlib import Path

DXF = Path.home() / "reference" / "quadruped" / "Lasercut_dxf_" / "Lasercut parts.dxf"


def entities(path):
    lines = path.read_text(errors="replace").splitlines()
    pairs = [(lines[i].strip(), lines[i + 1].strip()) for i in range(0, len(lines) - 1, 2)]
    start = next(k for k, (c, v) in enumerate(pairs) if c == "2" and v == "ENTITIES")
    out, cur = [], None
    for c, v in pairs[start + 1:]:
        if c == "0":
            if v == "ENDSEC":
                break
            cur = {"type": v}
            out.append(cur)
        else:
            cur.setdefault(c, v)
            if c in ("1", "3"):
                cur["text"] = cur.get("text", "") + v
    return out


def segments(ents, arc_step_deg=2.0):
    """Polyline pieces: each LINE or ARC as a list of (x, y) points."""
    segs = []
    for e in ents:
        if e["type"] == "LINE":
            segs.append([(float(e["10"]), float(e["20"])), (float(e["11"]), float(e["21"]))])
        elif e["type"] == "ARC":
            cx, cy, r = float(e["10"]), float(e["20"]), float(e["40"])
            a0, a1 = float(e["50"]), float(e["51"])
            if a1 < a0:
                a1 += 360.0
            n = max(2, int(math.ceil((a1 - a0) / arc_step_deg)) + 1)
            segs.append([(cx + r * math.cos(math.radians(a0 + (a1 - a0) * k / (n - 1))),
                          cy + r * math.sin(math.radians(a0 + (a1 - a0) * k / (n - 1)))) for k in range(n)])
    return segs


def loops(segs, tol=1e-3):
    """Join pieces end to end into closed loops."""
    segs = [list(s) for s in segs]
    out = []
    while segs:
        loop = segs.pop(0)
        while math.dist(loop[0], loop[-1]) > tol:
            for i, s in enumerate(segs):
                if math.dist(loop[-1], s[0]) <= tol:
                    loop += s[1:]
                elif math.dist(loop[-1], s[-1]) <= tol:
                    loop += s[::-1][1:]
                else:
                    continue
                segs.pop(i)
                break
            else:
                raise RuntimeError(f"open outline near {loop[-1]}")
        out.append(loop)
    return out


def area(poly):
    return abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))) / 2.0


def inside(pt, poly):
    x, y = pt
    c = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            c = not c
    return c


def measure(path=DXF):
    ents = entities(path)
    outlines = loops(segments(ents))
    holes = [(float(e["10"]), float(e["20"]), float(e["40"])) for e in ents if e["type"] == "CIRCLE"]
    plates = []
    for poly in sorted(outlines, key=area, reverse=True):
        xs, ys = [p[0] for p in poly], [p[1] for p in poly]
        h = [c for c in holes if inside(c[:2], poly)]
        # an outline inside a bigger outline is a cut-out of that plate, not a plate
        if any(inside(poly[0], q["outline"]) for q in plates):
            next(q for q in plates if inside(poly[0], q["outline"]))["cutouts_mm2"] += area(poly)
            continue
        plates.append({"outline": poly, "outline_mm2": area(poly), "cutouts_mm2": 0.0,
                       "holes_mm2": sum(math.pi * r * r for _, _, r in h), "n_holes": len(h),
                       "size_mm": (max(xs) - min(xs), max(ys) - min(ys))})
    for p in plates:
        p["net_mm2"] = p["outline_mm2"] - p["cutouts_mm2"] - p["holes_mm2"]
    notes = [e.get("text", "") for e in ents if e["type"] in ("MTEXT", "TEXT", "DIMENSION")]
    return plates, notes


def main():
    if not DXF.exists():
        sys.exit(f"{DXF} not found")
    plates, notes = measure()
    for k, p in enumerate(plates):
        print(f"plate {k}: {p['size_mm'][0]:.1f} x {p['size_mm'][1]:.1f} mm, outline {p['outline_mm2']:.0f} mm2, "
              f"cut-outs {p['cutouts_mm2']:.0f}, {p['n_holes']} holes {p['holes_mm2']:.0f} mm2 -> net {p['net_mm2']:.0f} mm2")
    print("text in the drawing:", notes)


if __name__ == "__main__":
    main()
