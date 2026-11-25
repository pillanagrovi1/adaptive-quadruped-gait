"""Stage 1 tests: the model is the measured robot, and it stands.

Run headless; needs no CAD files (only models/*.xml, models/*.json, docs/robot.md).
"""
import json
import math
import re
from pathlib import Path

import mujoco
import numpy as np
import pytest

from quad.model import LEGS, SERVOS, config, load
from quad.servo import ServoBank, joints_to_servos, servos_to_joints
from quad.stand import stand_test

ROOT = Path(__file__).resolve().parents[2]


def doc_numbers() -> dict:
    """The `| \\`key\\` | value | ...` rows of the 'Model numbers' table in docs/robot.md."""
    text = (ROOT / "docs" / "robot.md").read_text()
    return {k: float(v) for k, v in re.findall(r"^\|\s*`(\w+)`\s*\|\s*([-+\d.]+)\s*\|", text, re.M)}


@pytest.fixture(scope="module")
def model():
    return load()[0]


def test_loads_with_12_hinges(model):
    hinges = [model.joint(i).name for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    assert len(hinges) == 12
    assert sorted(hinges) == sorted(SERVOS)
    assert model.nu == 12


def test_lengths_match_docs(model):
    doc = doc_numbers()
    mm = lambda v: float(np.linalg.norm(v)) * 1000  # noqa: E731
    for leg in LEGS:
        K = model.body(f"{leg}_tibia").pos
        Fc = model.geom(f"{leg}_foot").pos
        r = float(model.geom(f"{leg}_foot").size[0])
        hip = model.body(f"{leg}_hip").pos
        got = {
            "femur_mm": mm(K[[0, 2]]),
            "tibia_knee_to_foot_mm": mm(Fc[[0, 2]]) + r * 1000,
            "foot_radius_mm": r * 1000,
            "roll_to_pitch_lateral_mm": abs(model.body(f"{leg}_femur").pos[1]) * 1000,
            "hip_x_front_mm" if leg[0] == "F" else "hip_x_rear_mm": hip[0] * 1000,
            "hip_y_mm": abs(hip[1]) * 1000,
            "hip_z_mm": hip[2] * 1000,
        }
        for k, v in got.items():
            assert abs(v - doc[k]) < 1.0, f"{leg} {k}: model {v:.2f} mm, docs/robot.md {doc[k]} mm"
    box = model.geom("torso").size * 2000
    assert abs(box[0] - doc["body_length_mm"]) < 1.0
    assert abs(box[1] - doc["body_width_mm"]) < 1.0
    assert abs(box[2] - doc["body_height_mm"]) < 1.0


def test_total_mass_matches_table(model):
    table = json.loads((ROOT / "models" / "mass_measured.json").read_text())
    model_g = float(model.body_subtreemass[model.body("torso").id]) * 1000
    assert abs(model_g - table["total_g"]) < 0.5
    assert abs(model_g - doc_numbers()["total_mass_g"]) < 0.5
    parts_g = sum(r["mass_g"] for r in table["parts"])
    assert abs(parts_g - table["total_g"]) < 0.5


def test_servo_limits_from_config(model):
    sv = config()["servo"]
    frc = sv["stall_torque_Nm"]["value"] * sv["torque_margin"]["value"]
    assert np.allclose(model.actuator_forcerange, [[-frc, frc]] * 12, atol=1e-3)
    assert all(math.degrees(r[1] - r[0]) <= 180.0 + 0.01 for r in model.actuator_ctrlrange)   # ranges stored to 1e-4 rad


def test_knee_servo_is_hip_plus_knee():
    q = np.radians(np.arange(12.0))
    assert np.allclose(servos_to_joints(joints_to_servos(q)), q)
    assert np.isclose(joints_to_servos(q)[2], q[1] + q[2])


def test_servo_rate_and_deadband(model):
    d = mujoco.MjData(model)
    sb = ServoBank(model, delay_ms=0)
    sb.reset(np.zeros(12))
    sb.command(np.full(12, 0.5))
    sb.step(d)
    assert np.allclose(sb.setpoint, sb.max_step)                 # rate limited
    sb.reset(np.zeros(12))
    sb.command(np.full(12, math.radians(0.2)))                   # inside the 0.36 deg deadband
    for _ in range(5):
        sb.step(d)
    assert np.allclose(sb.setpoint, 0)


def test_delay_buffer(model):
    d = mujoco.MjData(model)
    sb = ServoBank(model, delay_ms=20)
    sb.reset(np.zeros(12))
    sb.command(np.full(12, 0.01))
    steps = 0
    while np.allclose(sb.setpoint, 0):
        sb.step(d)
        steps += 1
    assert steps == sb.delay_steps + 1 == 21


@pytest.mark.parametrize("delay_ms", config()["delay_ms"]["sweep"])
def test_stand_10s(delay_ms):
    r = stand_test(delay_ms, seconds=10.0)
    assert max(r["torque_max_frac_stall"].values()) < 0.5
    assert not r["self_collisions"]
    assert not r["non_foot_floor_contacts"]
    assert abs(r["height_drift_1s_to_end_mm"]) < 2.0
    assert r["tilt_max_deg"] < 2.0
