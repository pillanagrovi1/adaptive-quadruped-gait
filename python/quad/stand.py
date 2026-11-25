"""Standing pose from inverse kinematics, and the Stage 1 stand test.

Standing pose: hip roll 0, each foot directly below its hip pitch axis, hip pitch axis at
stand_hip_height_mm above the floor (configs/robot.json). The leg is a 2-link chain in its own
plane: hip pitch -> knee (the femur vector) -> foot sphere centre (the tibia vector), both read
from the loaded model, so the IK always matches the MJCF.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from .model import LEGS, SERVOS, config, joint_qpos_adr, load
from .sensors import Sensors
from .servo import ServoBank, joints_to_servos, profile


def _ry(v, t):
    c, s = math.cos(t), math.sin(t)
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])   # (x, z) rotated about +y


def _ang(w) -> float:
    """Angle of an (x, z) vector such that rotating it by t about +y adds t (see _ry)."""
    return math.atan2(w[0], w[1])


def leg_ik(u: np.ndarray, v: np.ndarray, target: np.ndarray) -> list[tuple[float, float]]:
    """Both (hip pitch, knee) solutions (rad) with R(qh) (u + R(qk) v) = target.
    u: knee in the femur frame, v: foot centre in the tibia frame, target: foot centre in the hip frame; all (x, z)."""
    du, dv, dt = np.linalg.norm(u), np.linalg.norm(v), np.linalg.norm(target)
    c = (dt * dt - du * du - dv * dv) / (2 * du * dv)
    if abs(c) > 1:
        raise ValueError("target out of reach")
    out = []
    for g in (math.acos(c), -math.acos(c)):
        qk = _ang(u) - _ang(v) + g
        qh = _ang(target) - _ang(u + _ry(v, qk))
        out.append((math.remainder(qh, 2 * math.pi), math.remainder(qk, 2 * math.pi)))
    return out


def stand_pose(m: mujoco.MjModel, height_mm: float | None = None) -> np.ndarray:
    """12 joint angles (roll, pitch, knee per leg) for the standing pose."""
    h = (config()["stand_hip_height_mm"]["value"] if height_mm is None else height_mm) / 1000
    q = []
    for leg in LEGS:
        K = m.body(f"{leg}_tibia").pos
        Fc = m.geom(f"{leg}_foot").pos
        r = float(m.geom(f"{leg}_foot").size[0])
        u, v = np.array([K[0], K[2]]), np.array([Fc[0], Fc[2]])
        jr = m.joint(f"{leg}_knee").range
        best = None
        for qh, qk in leg_ik(u, v, np.array([0.0, -(h - r)])):
            if jr[0] <= qk <= jr[1]:
                best = (qh, qk)
        if best is None:
            raise ValueError(f"{leg}: no standing solution inside the knee range")
        q += [0.0, *best]
    return np.array(q)


def place(m: mujoco.MjModel, d: mujoco.MjData, q: np.ndarray, height_mm: float | None = None) -> None:
    """Put the robot in pose q with the torso level and the hips at the standing height."""
    h = (config()["stand_hip_height_mm"]["value"] if height_mm is None else height_mm) / 1000
    mujoco.mj_resetData(m, d)
    d.qpos[:3] = [0, 0, h]
    d.qpos[3:7] = [1, 0, 0, 0]
    d.qpos[joint_qpos_adr(m)] = q
    mujoco.mj_forward(m, d)
    # floor contact: lower the robot so the lowest foot sphere just touches z = 0
    low = min(d.geom_xpos[m.geom(f"{leg}_foot").id][2] - m.geom(f"{leg}_foot").size[0] for leg in LEGS)
    d.qpos[2] -= low
    mujoco.mj_forward(m, d)


def tilt_deg(quat: np.ndarray) -> float:
    """Angle between the torso's up axis and world up."""
    w, x, y, z = quat
    zz = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


def stand_test(delay_ms: float | None = None, seconds: float = 10.0, seed: int = 0,
               servo_profile: str | None = None) -> dict:
    """10 s on flat ground holding the standing pose. Returns the gate numbers."""
    cfg = config()
    prof = profile(servo_profile)
    m, d = load(prof["name"])
    q = stand_pose(m)
    place(m, d, q)
    servos = ServoBank(m, delay_ms, cfg, prof)
    s0 = joints_to_servos(q)
    servos.reset(s0)
    servos.command(s0)
    sens = Sensors(m, seed, cfg, prof=prof)
    stall, rated = prof["stall_torque_Nm"], prof["rated_torque_Nm"]
    steps = int(round(seconds / m.opt.timestep))
    robot = {m.geom(i).name for i in range(m.ngeom)} - {"floor"}
    feet = {f"{leg}_foot" for leg in LEGS}
    tau_max = np.zeros(12)
    tau_late = np.zeros(12)
    z, tilt, self_hits, floor_hits = [], [], set(), set()
    for k in range(steps):
        servos.step(d)
        mujoco.mj_step(m, d)
        tau = np.abs(servos.torques(d))
        tau_max = np.maximum(tau_max, tau)
        if k * m.opt.timestep >= 1.0:
            tau_late = np.maximum(tau_late, tau)
        for c in d.contact[:d.ncon]:
            a, b = m.geom(c.geom1).name, m.geom(c.geom2).name
            if a in robot and b in robot:
                self_hits.add(tuple(sorted((a, b))))
            elif "floor" in (a, b) and (a if b == "floor" else b) not in feet:
                floor_hits.add(a if b == "floor" else b)
        tr = sens.truth(d)
        z.append(tr["height"])
        tilt.append(tilt_deg(tr["quat"]))
    z = np.array(z)
    i1 = int(round(1.0 / m.opt.timestep))
    return {
        "servo_profile": prof["name"], "stall_torque_Nm": stall, "rated_torque_Nm": rated,
        "delay_ms": servos.delay_ms, "seconds": seconds,
        "hip_height_cmd_mm": cfg["stand_hip_height_mm"]["value"],
        "torso_z_start_mm": float(z[0] * 1000), "torso_z_1s_mm": float(z[i1] * 1000), "torso_z_end_mm": float(z[-1] * 1000),
        "height_drift_1s_to_end_mm": float((z[-1] - z[i1]) * 1000),
        "settle_drop_mm": float((z[0] - z[i1]) * 1000),
        "tilt_max_deg": float(max(tilt)), "tilt_end_deg": float(tilt[-1]),
        "torque_max_Nm": dict(zip(SERVOS, tau_max.round(4).tolist())),
        "torque_max_frac_stall": dict(zip(SERVOS, (tau_max / stall).round(4).tolist())),
        "torque_after_1s_frac_stall": dict(zip(SERVOS, (tau_late / stall).round(4).tolist())),
        "torque_max_frac_rated": dict(zip(SERVOS, (tau_max / rated).round(4).tolist())),
        "torque_after_1s_frac_rated": dict(zip(SERVOS, (tau_late / rated).round(4).tolist())),
        "self_collisions": sorted(self_hits), "non_foot_floor_contacts": sorted(floor_hits),
        "stand_pose_rad": dict(zip(SERVOS, q.round(5).tolist())),
    }
