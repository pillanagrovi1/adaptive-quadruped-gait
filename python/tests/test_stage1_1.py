"""Stage 1.1 tests: servo profiles, step and delay response, joint feedback, backlash.

Headless; needs no CAD files.
"""
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from quad.model import SERVOS, config, joint_qpos_adr, load
from quad.sensors import Sensors
from quad.servo import ServoBank, joints_to_servos, profile, top_speed
from quad.stand import place, stand_pose
from quad.step_test import step_response

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ("sts3215_7v4", "sts3215_12v", "spt5435_guess")
STATUS = {"measured", "published", "derived", "assumed"}


def test_every_profile_value_has_a_status_and_source():
    allp = json.loads((ROOT / "configs" / "servos.json").read_text())
    assert config()["servo_profile"]["value"] == "sts3215_7v4"
    for name in PROFILES:
        for k, v in allp[name].items():
            if isinstance(v, dict):
                assert v["status"] in STATUS, f"{name}.{k}"
                assert v["source"], f"{name}.{k}"


@pytest.mark.parametrize("name", PROFILES)
def test_step_reaches_no_load_speed(name):
    """60 deg step, no load: peak speed within 15 % of the profile's own no-load speed."""
    r = step_response(name)
    assert abs(r["peak_speed_rad_s"] / r["no_load_speed_rad_s"] - 1) <= 0.15
    assert abs(r["peak_speed_rad_s"] / top_speed(profile(name)) - 1) <= 0.05    # set by damping + clip, nothing else
    assert r["t95_s"] < 0.5


@pytest.mark.parametrize("name", PROFILES)
def test_delay_shifts_the_start(name):
    cfg = config()
    tol = cfg["timestep_s"]["value"] + 1.0 / cfg["control_hz"]["value"]
    t0 = step_response(name, 0.0, seconds=0.15)["t_start_s"]
    for dl in cfg["delay_ms"]["sweep"]:
        t = step_response(name, dl, seconds=0.15)["t_start_s"]
        assert abs((t - t0) - dl / 1000) <= tol, f"{dl} ms: started {1000 * (t - t0):.1f} ms later"


@pytest.mark.parametrize("name", PROFILES)
def test_profile_loads_into_the_robot(name):
    p = profile(name)
    m, _ = load(name)
    assert np.allclose(m.actuator_gainprm[:, 0], p["kp_Nm_per_rad"])
    assert np.allclose(m.actuator_forcerange, [[-p["forcerange_Nm"], p["forcerange_Nm"]]] * 12)
    assert np.allclose(m.tendon_damping, p["damping_Nm_s_per_rad"])
    assert np.allclose(m.actuator_biasprm[:, 2], 0)       # kv = 0: back-EMF is damping, not in the actuator (forcerange would clip it)


def test_servo_mass_is_the_default_profile():
    table = json.loads((ROOT / "models" / "mass_measured.json").read_text())
    servos = [r for r in table["parts"] if r["part"] == "spt5435LV"]
    assert len(servos) == 12
    assert all(r["mass_g"] == profile()["mass_g"] for r in servos)


@pytest.fixture(scope="module")
def standing():
    m, d = load()
    q = stand_pose(m)
    place(m, d, q)
    sb = ServoBank(m)
    sb.reset(joints_to_servos(q))
    for _ in range(500):
        sb.step(d)
        mujoco.mj_step(m, d)
    return m, d


def test_observe_shapes_and_units(standing):
    m, d = standing
    o = Sensors(m, seed=1).observe(d)
    assert set(o) == {"gyro", "accel", "contact", "joint_angle", "joint_speed", "load_frac"}
    assert o["gyro"].shape == (3,) and o["accel"].shape == (3,) and o["contact"].shape == (4,)
    for k in ("joint_angle", "joint_speed", "load_frac"):
        assert o[k].shape == (12,)
    true = joints_to_servos(d.qpos[joint_qpos_adr(m)])
    assert np.max(np.abs(o["joint_angle"] - true)) < 0.01            # rad, knee = tibia pitch
    assert np.max(np.abs(o["load_frac"])) < 0.5                      # a fraction of stall, not N.m
    assert np.all(o["contact"])


def test_joint_angle_is_quantised():
    cfg = config()
    cfg["joint_sensors"]["angle_noise_rad"]["value"] = 0.0
    m, d = load()
    d.qpos[joint_qpos_adr(m)] = 0.001234
    a = Sensors(m, cfg=cfg).read_joints(d)["joint_angle"]
    step = 2 * math.pi / 4096
    assert np.allclose(a / step, np.round(a / step))


def test_joint_readings_arrive_one_tick_late():
    m, d = load()
    s = Sensors(m)
    first = s.observe(d)["joint_angle"]
    d.qpos[joint_qpos_adr(m)] = 0.3
    late = s.observe(d)["joint_angle"]                  # still the old reading
    now = s.observe(d)["joint_angle"]
    assert np.max(np.abs(late - first)) < 0.02
    assert np.min(np.abs(now - first)) > 0.2


def test_truth_never_in_observe(standing):
    m, d = standing
    s = Sensors(m, seed=2)
    o, tr = s.observe(d), s.truth(d)
    assert not set(o) & set(tr)
    flat = np.concatenate([np.ravel(np.asarray(v, float)) for v in o.values()])
    for v in (tr["quat"], tr["position"], [tr["height"]]):
        for x in np.ravel(v):
            if abs(x) > 1e-3:
                assert not np.any(np.isclose(flat, x, rtol=0, atol=1e-9)), f"truth value {x} found in observe()"


def test_backlash_option():
    m, _ = load(backlash=True)
    bl = [m.joint(i).name for i in range(m.njnt) if m.joint(i).name.endswith("_backlash")]
    assert len(bl) == 12
    r = math.radians(profile()["backlash_deg"])
    assert all(np.allclose(m.joint(n).range, [-r, r]) for n in bl)
    assert load()[0].njnt == 13                          # off by default
    assert SERVOS == tuple(n.removesuffix("_backlash") for n in bl)
