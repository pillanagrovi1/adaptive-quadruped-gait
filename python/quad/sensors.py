"""What the robot can sense: IMU (gyro + accelerometer) with noise and bias, and foot contacts.

The learner may see `observe()` and nothing else. True orientation (`truth()`) is for logs and
plots only (rule in docs/architecture.md).
"""
from __future__ import annotations

import mujoco
import numpy as np

from .model import LEGS, config


class Sensors:
    def __init__(self, m: mujoco.MjModel, seed: int = 0, cfg: dict | None = None, contact_N: float = 0.5):
        imu = (cfg or config())["imu"]
        self.m = m
        self.rng = np.random.default_rng(seed)
        self.gn, self.gb = imu["gyro_noise_rad_s"]["value"], imu["gyro_bias_rad_s"]["value"]
        self.an, self.ab = imu["accel_noise_m_s2"]["value"], imu["accel_bias_m_s2"]["value"]
        self.contact_N = contact_N
        adr = lambda n: (int(m.sensor(n).adr[0]), int(m.sensor(n).dim[0]))  # noqa: E731
        self.gyro, self.accel, self.quat = adr("imu_gyro"), adr("imu_accel"), adr("true_orientation")
        self.touch = [adr(f"{leg}_contact") for leg in LEGS]
        self.reset()

    def reset(self) -> None:
        """New constant biases, as after a power cycle."""
        self.gyro_bias = self.rng.normal(0, self.gb, 3)
        self.accel_bias = self.rng.normal(0, self.ab, 3)

    def _get(self, d, a):
        return d.sensordata[a[0]:a[0] + a[1]].copy()

    def observe(self, d: mujoco.MjData) -> dict:
        """Learner-visible signals."""
        return {
            "gyro": self._get(d, self.gyro) + self.gyro_bias + self.rng.normal(0, self.gn, 3),
            "accel": self._get(d, self.accel) + self.accel_bias + self.rng.normal(0, self.an, 3),
            "contact": np.array([self._get(d, t)[0] > self.contact_N for t in self.touch]),
        }

    def truth(self, d: mujoco.MjData) -> dict:
        """Analysis only. Never a learner input."""
        return {"quat": self._get(d, self.quat), "height": float(d.qpos[2])}
