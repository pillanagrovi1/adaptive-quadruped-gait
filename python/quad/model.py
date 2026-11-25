"""Load models/quadruped.xml and name its parts.

    from quad.model import load, LEGS
    m, d = load()
"""
from __future__ import annotations

import json
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[2]
XML = ROOT / "models" / "quadruped.xml"
CONFIG = ROOT / "configs" / "robot.json"
LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("hip_roll", "hip_pitch", "knee")
# servo order = actuator order: per leg hip roll, hip pitch, knee servo (= tibia pitch, see docs/robot.md)
SERVOS = tuple(f"{leg}_{j}" for leg in LEGS for j in JOINTS)


def config() -> dict:
    return json.loads(CONFIG.read_text())


def load(xml: Path = XML) -> tuple[mujoco.MjModel, mujoco.MjData]:
    m = mujoco.MjModel.from_xml_path(str(xml))
    return m, mujoco.MjData(m)


def joint_qpos_adr(m: mujoco.MjModel) -> list[int]:
    """qpos index of each leg joint, in SERVOS order."""
    return [int(m.joint(n).qposadr[0]) for n in SERVOS]
