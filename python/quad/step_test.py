"""Servo step test: one servo in the air, no gravity, no load, a 60 deg step.

    light link (20 g, 50 mm) on one hinge, driven by one servo of a profile, through ServoBank
    (so the delay and deadband are the robot's own)

It reports the peak speed and the time to reach 95 % of the step. The peak speed must be close
to the profile's own no-load speed: if a speed limit were counted twice (Stage 1's bug, D17),
the servo would be much slower than its spec.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from .model import config
from .servo import ServoBank, profile

RIG = """
<mujoco model="servo_rig">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt}" gravity="0 0 0" integrator="implicitfast"/>
  <worldbody>
    <body name="link">
      <joint name="servo" axis="0 1 0" damping="{damp}" armature="{arm}" frictionloss="{fl}"/>
      <geom type="capsule" fromto="0 0 0 0.05 0 0" size="0.006" mass="0.02"/>
    </body>
  </worldbody>
  <actuator>
    <position name="servo" joint="servo" kp="{kp}" forcerange="-{frc} {frc}" ctrlrange="-3.2 3.2"/>
  </actuator>
</mujoco>
"""


def rig(prof: dict) -> tuple[mujoco.MjModel, mujoco.MjData]:
    m = mujoco.MjModel.from_xml_string(RIG.format(
        dt=config()["timestep_s"]["value"], kp=prof["kp_Nm_per_rad"], frc=prof["forcerange_Nm"],
        damp=prof["damping_Nm_s_per_rad"], arm=prof["armature_kg_m2"], fl=prof["frictionloss_Nm"]))
    return m, mujoco.MjData(m)


def step_response(name: str | None = None, delay_ms: float = 0.0, step_deg: float = 60.0,
                  seconds: float = 0.6) -> dict:
    """The command jumps from 0 to step_deg at t = 0. Returns the trace and its numbers."""
    prof = profile(name)
    m, d = rig(prof)
    sb = ServoBank(m, delay_ms=delay_ms, prof=prof, neutral_offset=0.0, actuators=[0])
    sb.reset(np.zeros(1))
    target = math.radians(step_deg)
    sb.command(np.array([target]))
    n = int(round(seconds / m.opt.timestep))
    t, q, v = np.zeros(n), np.zeros(n), np.zeros(n)
    for k in range(n):
        sb.step(d)
        mujoco.mj_step(m, d)
        t[k], q[k], v[k] = d.time, d.qpos[0], d.qvel[0]
    moving = np.nonzero(q > math.radians(0.1))[0]
    reached = np.nonzero(q >= 0.95 * target)[0]
    return {
        "profile": prof["name"], "delay_ms": delay_ms, "step_deg": step_deg,
        "peak_speed_rad_s": float(v.max()),
        "no_load_speed_rad_s": prof["no_load_speed_rad_s"],
        "model_top_speed_rad_s": (prof["forcerange_Nm"] - prof["frictionloss_Nm"]) / prof["damping_Nm_s_per_rad"],
        "t_start_s": float(t[moving[0]]) if len(moving) else math.nan,
        "t95_s": float(t[reached[0]]) if len(reached) else math.nan,
        "overshoot_deg": float(math.degrees(q.max() - target)),
        "t": t, "q": q, "v": v,
    }
