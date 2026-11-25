#!/usr/bin/env python3
"""Renders for docs/img: the standing pose seen roughly from where photo 20201107_124135.jpg was
taken (robot's right side, a little from the front, camera ~20 deg up), and from the front.

    MUJOCO_GL=egl python3 scripts/render_stage1.py
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

from quad.model import load  # noqa: E402
from quad.stand import place, stand_pose  # noqa: E402

VIEWS = {
    # azimuth: direction the camera looks, measured in the floor plane (MuJoCo convention, deg)
    "stage1_side": {"azimuth": 100.0, "elevation": -20.0, "distance": 0.62},
    "stage1_front": {"azimuth": 180.0, "elevation": -15.0, "distance": 0.55},
}


def main():
    m, d = load()
    place(m, d, stand_pose(m))
    r = mujoco.Renderer(m, 720, 1280)
    out = ROOT / "docs" / "img"
    out.mkdir(parents=True, exist_ok=True)
    for name, v in VIEWS.items():
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0.0, 0.0, 0.09]
        cam.azimuth, cam.elevation, cam.distance = v["azimuth"], v["elevation"], v["distance"]
        r.update_scene(d, cam)
        Image.fromarray(r.render()).save(out / f"{name}.png")
        print("wrote", (out / f"{name}.png").relative_to(ROOT))


if __name__ == "__main__":
    main()
