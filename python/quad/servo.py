"""Hobby-servo model: a thin layer around MuJoCo's position actuators.

What one servo does, every physics step (numbers in configs/robot.json):

    command (servo angle) --delay--> received --deadband--> setpoint --rate limit--> ctrl
                                                                                      |
    MuJoCo position actuator:  torque = kp * (ctrl - angle), clipped to +-margin * stall
    joint (or knee tendon) damping = back-EMF;  armature = gear inertia

- delay:     a ring buffer of commands, delay_ms long (default 20 ms, a guess)
- deadband:  a new command closer than deadband_deg to the last one is ignored
- rate:      the setpoint moves towards the received command at most max_rate_rad_s
The knee servo's angle is the tibia's pitch (hip pitch + knee), because the push-rods are two
parallelograms (docs/robot.md). `joints_to_servos` converts model joint angles to servo angles.
"""
from __future__ import annotations

import math
from collections import deque

import mujoco
import numpy as np

from .model import JOINTS, LEGS, config


def joints_to_servos(q: np.ndarray) -> np.ndarray:
    """12 joint angles (roll, pitch, knee per leg) -> 12 servo angles (knee servo = pitch + knee)."""
    s = np.array(q, dtype=float).reshape(4, 3).copy()
    s[:, 2] = s[:, 1] + s[:, 2]
    return s.reshape(12)


def servos_to_joints(s: np.ndarray) -> np.ndarray:
    q = np.array(s, dtype=float).reshape(4, 3).copy()
    q[:, 2] = q[:, 2] - q[:, 1]
    return q.reshape(12)


class ServoBank:
    def __init__(self, m: mujoco.MjModel, delay_ms: float | None = None, cfg: dict | None = None):
        cfg = cfg or config()
        sv = cfg["servo"]
        self.m = m
        self.dt = float(m.opt.timestep)
        self.delay_ms = float(cfg["delay_ms"]["value"] if delay_ms is None else delay_ms)
        self.delay_steps = int(round(self.delay_ms / 1000 / self.dt))
        self.max_step = sv["max_rate_rad_s"]["value"] * self.dt
        self.deadband = math.radians(sv["deadband_deg"]["value"])
        self.act = [m.actuator(f"{leg}_{j}").id for leg in LEGS for j in JOINTS]
        self.lo = m.actuator_ctrlrange[self.act, 0].copy()
        self.hi = m.actuator_ctrlrange[self.act, 1].copy()
        self.reset(np.zeros(12))

    def reset(self, servo_angles: np.ndarray) -> None:
        """Start at rest: every stage of the pipeline holds these angles."""
        s = np.clip(np.asarray(servo_angles, float), self.lo, self.hi)
        self.queue = deque([s.copy() for _ in range(self.delay_steps + 1)], maxlen=self.delay_steps + 1)
        self.received = s.copy()
        self.setpoint = s.copy()
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
        self.setpoint += np.clip(self.received - self.setpoint, -self.max_step, self.max_step)
        d.ctrl[self.act] = self.setpoint

    def torques(self, d: mujoco.MjData) -> np.ndarray:
        """Torque each servo delivers now (N.m)."""
        return d.actuator_force[self.act].copy()
