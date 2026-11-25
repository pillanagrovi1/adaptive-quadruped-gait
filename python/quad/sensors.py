"""What the robot can feel: IMU, foot contacts, and each servo's angle, speed and load.

The learner may see `observe()` and nothing else. True orientation, height, position and terrain
(`truth()`) are for logs and plots only (rule in docs/architecture.md).

Joint feedback is what a real STS3215 reports over its bus (numbers in configs/robot.json):
- angle    (rad): the servo's angle, quantised to 4096 steps per turn, plus noise. The knee servo
                  reports the tibia's pitch (hip pitch + knee), because that is what it measures.
- speed    (rad/s): the servo's speed, plus noise.
- load_frac (-): actuator force / the profile's stall torque, plus noise. On the real servo this
                  is a drive duty cycle, NOT a torque sensor; hence the name.
All three arrive `read_delay_ticks` control ticks late (default 1): observe() returns what was
read on an earlier call. Call observe() once per control tick.
"""
from __future__ import annotations

import math
from collections import deque

import mujoco
import numpy as np

from .model import LEGS, SERVOS, config, joint_qpos_adr
from .servo import joints_to_servos, profile


class Sensors:
    def __init__(self, m: mujoco.MjModel, seed: int = 0, cfg: dict | None = None, contact_N: float = 0.5,
                 prof: dict | None = None, neutral_offset: np.ndarray | float | None = None):
        cfg = cfg or config()
        imu, js = cfg["imu"], cfg["joint_sensors"]
        self.m = m
        self.rng = np.random.default_rng(seed)
        self.gn, self.gb = imu["gyro_noise_rad_s"]["value"], imu["gyro_bias_rad_s"]["value"]
        self.an, self.ab = imu["accel_noise_m_s2"]["value"], imu["accel_bias_m_s2"]["value"]
        self.step_rad = 2 * math.pi / js["angle_steps_per_turn"]["value"]
        self.qn, self.vn = js["angle_noise_rad"]["value"], js["speed_noise_rad_s"]["value"]
        self.ln = js["load_noise_frac"]["value"]
        self.read_delay = int(js["read_delay_ticks"]["value"])
        self.stall = (prof or profile())["stall_torque_Nm"]
        off = cfg["neutral_offset_rad"]["value"] if neutral_offset is None else neutral_offset
        self.offset = np.broadcast_to(np.asarray(off, float), (12,)).copy()
        self.contact_N = contact_N
        adr = lambda n: (int(m.sensor(n).adr[0]), int(m.sensor(n).dim[0]))  # noqa: E731
        self.gyro, self.accel, self.quat = adr("imu_gyro"), adr("imu_accel"), adr("true_orientation")
        self.touch = [adr(f"{leg}_contact") for leg in LEGS]
        self.qadr = joint_qpos_adr(m)
        self.vadr = [int(m.joint(n).dofadr[0]) for n in SERVOS]
        self.act = [m.actuator(n).id for n in SERVOS]
        self.reset()

    def reset(self) -> None:
        """New constant IMU biases, as after a power cycle; empty joint-read pipeline."""
        self.gyro_bias = self.rng.normal(0, self.gb, 3)
        self.accel_bias = self.rng.normal(0, self.ab, 3)
        self.pending: deque = deque()

    def _get(self, d, a):
        return d.sensordata[a[0]:a[0] + a[1]].copy()

    def read_joints(self, d: mujoco.MjData) -> dict:
        """One bus read now, with no delay (observe() delays it)."""
        angle = joints_to_servos(d.qpos[self.qadr]) - self.offset       # the servo measures from its own zero
        angle = np.round(angle / self.step_rad) * self.step_rad + self.rng.normal(0, self.qn, 12)
        speed = joints_to_servos(d.qvel[self.vadr]) + self.rng.normal(0, self.vn, 12)
        load = d.actuator_force[self.act] / self.stall + self.rng.normal(0, self.ln, 12)
        return {"joint_angle": angle, "joint_speed": speed, "load_frac": load}

    def observe(self, d: mujoco.MjData) -> dict:
        """Learner-visible signals. Shapes: gyro (3,) rad/s, accel (3,) m/s2, contact (4,) bool,
        joint_angle (12,) rad, joint_speed (12,) rad/s, load_frac (12,) fraction of stall; servo order SERVOS."""
        self.pending.append(self.read_joints(d))
        while len(self.pending) > self.read_delay + 1:
            self.pending.popleft()
        joints = self.pending[0]                       # read_delay ticks old (the oldest until the pipe fills)
        return {
            "gyro": self._get(d, self.gyro) + self.gyro_bias + self.rng.normal(0, self.gn, 3),
            "accel": self._get(d, self.accel) + self.accel_bias + self.rng.normal(0, self.an, 3),
            "contact": np.array([self._get(d, t)[0] > self.contact_N for t in self.touch]),
            **{k: v.copy() for k, v in joints.items()},
        }

    def truth(self, d: mujoco.MjData) -> dict:
        """Analysis only. Never a learner input."""
        return {"quat": self._get(d, self.quat), "height": float(d.qpos[2]), "position": d.qpos[:3].copy()}
