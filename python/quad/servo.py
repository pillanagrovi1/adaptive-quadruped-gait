"""Servo model: profiles (configs/servos.json) on top of MuJoCo's position actuators.

What one servo does, every physics step:

    command (servo angle) --delay--> received --deadband--> + neutral offset --> ctrl
                                                                                  |
    MuJoCo position actuator:  torque = kp * (ctrl - angle), clipped to +-forcerange
    joint (or knee tendon): damping = back-EMF, frictionloss = gear friction, armature = motor inertia

There is no rate limiter: damping + the torque clip already set the top speed,
    top speed ~ (forcerange - frictionloss) / damping        (docs/study/stage1_1.md)
(Stage 1 also had a setpoint rate limiter; it counted the speed limit twice, decision D17.)

- delay:    a ring buffer of commands, delay_ms long (default 20 ms, a guess)
- deadband: a new command closer than deadband_deg to the last one is ignored
- offset:   the servo's real zero is off by neutral_offset_rad (default 0)
The knee servo's angle is the tibia's pitch (hip pitch + knee), because the push-rods are two
parallelograms (docs/robot.md). `joints_to_servos` converts model joint angles to servo angles.
"""
from __future__ import annotations

import json
import math
from collections import deque

import mujoco
import numpy as np

from .model import JOINTS, LEGS, ROOT, config

SERVOS_JSON = ROOT / "configs" / "servos.json"


def profile(name: str | None = None) -> dict:
    """A servo profile as plain numbers {key: value}, plus its 'name'. Default: configs/robot.json."""
    name = name or config()["servo_profile"]["value"]
    p = json.loads(SERVOS_JSON.read_text())[name]
    out = {k: v["value"] for k, v in p.items() if isinstance(v, dict) and "value" in v}
    out["name"] = name
    return out


def top_speed(p: dict) -> float:
    """Unloaded top speed the model reaches (rad/s): torque limit minus friction, over damping."""
    return (p["forcerange_Nm"] - p["frictionloss_Nm"]) / p["damping_Nm_s_per_rad"]


def apply_profile(spec: mujoco.MjSpec, p: dict, backlash: bool = False) -> None:
    """Set every servo's actuator, joint and knee tendon to profile p (in place).

    Hip roll / hip pitch: damping, frictionloss, armature on the joint.
    Knee servo: on the tendon hip_pitch + knee (the knee joint itself has none).
    backlash=True adds, per servo, a free hinge of +-backlash_deg on the same axis to the body
    the servo moves (MuJoCo modelling guide; Open Duck's 'backlash' class)."""
    kp, frc = p["kp_Nm_per_rad"], p["forcerange_Nm"]
    for a in spec.actuators:
        a.gainprm[0] = kp
        a.biasprm[1] = -kp
        a.forcerange = [-frc, frc]
    for leg in LEGS:
        for j in ("hip_roll", "hip_pitch"):
            jt = spec.joint(f"{leg}_{j}")
            jt.damping[0], jt.frictionloss, jt.armature = p["damping_Nm_s_per_rad"], p["frictionloss_Nm"], p["armature_kg_m2"]
        t = spec.tendon(f"{leg}_knee_servo")
        t.damping[0], t.frictionloss, t.armature = p["damping_Nm_s_per_rad"], p["frictionloss_Nm"], p["armature_kg_m2"]
    if backlash:
        r = math.radians(p["backlash_deg"])
        for leg in LEGS:
            for body, j in (("hip", "hip_roll"), ("femur", "hip_pitch"), ("tibia", "knee")):
                main = spec.joint(f"{leg}_{j}")
                bl = spec.body(f"{leg}_{body}").add_joint()
                bl.name = f"{leg}_{j}_backlash"
                bl.type = mujoco.mjtJoint.mjJNT_HINGE
                bl.axis = main.axis
                bl.pos = main.pos
                bl.range = [-r, r]
                bl.limited = mujoco.mjtLimited.mjLIMITED_TRUE
                bl.damping[0] = p["backlash_damping_Nm_s_per_rad"]
                bl.armature = p["backlash_armature_kg_m2"]


def joints_to_servos(q: np.ndarray) -> np.ndarray:
    """12 joint angles (roll, pitch, knee per leg) -> 12 servo angles (knee servo = pitch + knee).
    Also works for speeds (it is linear)."""
    s = np.array(q, dtype=float).reshape(4, 3).copy()
    s[:, 2] = s[:, 1] + s[:, 2]
    return s.reshape(12)


def servos_to_joints(s: np.ndarray) -> np.ndarray:
    q = np.array(s, dtype=float).reshape(4, 3).copy()
    q[:, 2] = q[:, 2] - q[:, 1]
    return q.reshape(12)


class ServoBank:
    def __init__(self, m: mujoco.MjModel, delay_ms: float | None = None, cfg: dict | None = None,
                 prof: dict | None = None, neutral_offset: np.ndarray | float | None = None,
                 actuators: list[int] | None = None):
        """actuators: actuator ids to drive (default: the robot's 12, in SERVOS order)."""
        cfg = cfg or config()
        self.prof = prof or profile()
        self.m = m
        self.dt = float(m.opt.timestep)
        self.delay_ms = float(cfg["delay_ms"]["value"] if delay_ms is None else delay_ms)
        self.delay_steps = int(round(self.delay_ms / 1000 / self.dt))
        self.deadband = math.radians(self.prof["deadband_deg"])
        self.act = actuators or [m.actuator(f"{leg}_{j}").id for leg in LEGS for j in JOINTS]
        n = len(self.act)
        off = cfg["neutral_offset_rad"]["value"] if neutral_offset is None else neutral_offset
        self.offset = np.broadcast_to(np.asarray(off, float), (n,)).copy()
        self.lo = m.actuator_ctrlrange[self.act, 0].copy()
        self.hi = m.actuator_ctrlrange[self.act, 1].copy()
        self.reset(np.zeros(n))

    def reset(self, servo_angles: np.ndarray) -> None:
        """Start at rest: every stage of the pipeline holds these angles."""
        s = np.clip(np.asarray(servo_angles, float), self.lo, self.hi)
        self.queue = deque([s.copy() for _ in range(self.delay_steps + 1)], maxlen=self.delay_steps + 1)
        self.received = s.copy()
        self.command_now = s.copy()

    def command(self, servo_angles: np.ndarray) -> None:
        """What the controller sends (held until the next call), in servo angles (rad)."""
        self.command_now = np.clip(np.asarray(servo_angles, float), self.lo, self.hi)

    def step(self, d: mujoco.MjData) -> None:
        """Call once before every mj_step."""
        self.queue.append(self.command_now.copy())
        arrived = self.queue[0]
        moved = np.abs(arrived - self.received) >= self.deadband
        self.received = np.where(moved, arrived, self.received)
        d.ctrl[self.act] = self.received + self.offset

    def torques(self, d: mujoco.MjData) -> np.ndarray:
        """Torque each servo delivers now (N.m)."""
        return d.actuator_force[self.act].copy()
