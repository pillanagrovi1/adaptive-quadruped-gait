#!/usr/bin/env python3
"""Stage 1 gate (default servo profile): stand test at every delay value, plus the hand check of the static torques.

Writes results/stage1_stand.json (the numbers in docs/results.md).

Hand check (docs/study/stage1.md): a leg carrying a vertical foot force W, foot under the hip:
    knee servo  (turns the tibia about the knee):  tau ~ W x (horizontal distance knee -> foot)
    hip servo   (turns the femur about the hip):   tau ~ W x (horizontal distance hip -> knee)
W = m g / 4 for the quick estimate; also W from the front/rear split of the centre of mass.

    python3 scripts/stage1_report.py
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from quad.model import LEGS, config, load  # noqa: E402
from quad.servo import ServoBank, joints_to_servos  # noqa: E402
from quad.stand import place, stand_pose, stand_test  # noqa: E402

OUT = ROOT / "results" / "stage1_stand.json"


def hand_check() -> dict:
    cfg = config()
    m, d = load()
    # gear friction (frictionloss) can hold part of a static load, so the sim would not match a
    # friction-free hand calculation; switch it off for this check only
    m.dof_frictionloss[:] = 0
    m.tendon_frictionloss[:] = 0
    q = stand_pose(m)
    place(m, d, q)
    g = -m.opt.gravity[2]
    mass = float(m.body_subtreemass[m.body("torso").id])
    com = d.subtree_com[m.body("torso").id].copy()
    xf = np.mean([d.xpos[m.body(f"{l}_femur").id][0] for l in ("FL", "FR")])
    xr = np.mean([d.xpos[m.body(f"{l}_femur").id][0] for l in ("RL", "RR")])
    front_share = (com[0] - xr) / (xf - xr)               # moment balance about the rear feet line
    out = {"mass_kg": mass, "com_x_mm": com[0] * 1000, "front_share": front_share, "legs": {}}
    # simulated: hold the pose for 3 s, read the servo torques and the foot forces
    servos = ServoBank(m, cfg=cfg)
    servos.reset(joints_to_servos(q))
    for _ in range(3000):
        servos.step(d)
        mujoco.mj_step(m, d)
    tau = servos.torques(d).reshape(4, 3)
    for i, leg in enumerate(LEGS):
        H = d.xpos[m.body(f"{leg}_femur").id]
        K = d.xpos[m.body(f"{leg}_tibia").id]
        F = d.geom_xpos[m.geom(f"{leg}_foot").id]
        f6 = np.zeros(6)
        fz = 0.0
        for k, c in enumerate(d.contact[:d.ncon]):
            if m.geom(f"{leg}_foot").id in (c.geom1, c.geom2):
                mujoco.mj_contactForce(m, d, k, f6)
                fz += f6[0]                                   # normal force (the floor is flat)
        mt = float(m.body_subtreemass[m.body(f"{leg}_tibia").id])
        mf = float(m.body_mass[m.body(f"{leg}_femur").id])
        ct = d.subtree_com[m.body(f"{leg}_tibia").id]
        cf = d.xipos[m.body(f"{leg}_femur").id]
        W4 = mass * g / 4
        Wfr = mass * g * (front_share if leg[0] == "F" else 1 - front_share) / 2
        out["legs"][leg] = {
            "lever_hip_mm": abs(K[0] - H[0]) * 1000, "lever_knee_mm": abs(F[0] - K[0]) * 1000,
            "W_quarter_N": W4, "W_front_rear_N": Wfr, "W_sim_N": fz,
            "hand_hip_Nm": W4 * abs(K[0] - H[0]), "hand_knee_Nm": W4 * abs(F[0] - K[0]),
            "hand_fr_hip_Nm": Wfr * abs(K[0] - H[0]), "hand_fr_knee_Nm": Wfr * abs(F[0] - K[0]),
            # refined: the foot force W_sim, minus the leg's own weight on its lever. The knee servo holds the
            # tibia's angle (push-rods), so the hip servo feels the tibia's weight at the knee K, not at its centre
            # of mass (Stage 1 used ct here: that was the 2.5 % gap, docs/results.md)
            "refined_hip_Nm": abs(fz * (K[0] - H[0]) - mt * g * (K[0] - H[0]) - mf * g * (cf[0] - H[0])),
            "refined_hip_stage1_formula_Nm": abs(fz * (K[0] - H[0]) - mt * g * (ct[0] - H[0]) - mf * g * (cf[0] - H[0])),
            "refined_knee_Nm": abs(fz * (F[0] - K[0]) - mt * g * (ct[0] - K[0])),
            "sim_hip_Nm": float(abs(tau[i, 1])), "sim_knee_Nm": float(abs(tau[i, 2])), "sim_roll_Nm": float(abs(tau[i, 0])),
        }
    return out


def main():
    cfg = config()
    runs = [stand_test(dl) for dl in cfg["delay_ms"]["sweep"]]
    stall, rated = runs[0]["stall_torque_Nm"], runs[0]["rated_torque_Nm"]
    for r in runs:
        r["gate_pass"] = (max(r["torque_max_frac_stall"].values()) < 0.5 and not r["self_collisions"]
                          and not r["non_foot_floor_contacts"])
        r["above_half_rated"] = [k for k, v in r["torque_max_frac_rated"].items() if v > 0.5]
    hc = hand_check()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({"servo_profile": runs[0]["servo_profile"], "stall_torque_Nm": stall,
                               "rated_torque_Nm": rated, "runs": runs, "hand_check": hc}, indent=1))
    for r in runs:
        fr = r["torque_max_frac_stall"]
        print(f"delay {r['delay_ms']:4.0f} ms: max {max(fr.values()):.3f} of stall ({max(fr, key=fr.get)}), "
              f"after 1 s {max(r['torque_after_1s_frac_stall'].values()):.3f}, settle {r['settle_drop_mm']:.2f} mm, "
              f"of rated {max(r['torque_max_frac_rated'].values()):.3f}, drift {r['height_drift_1s_to_end_mm']:+.4f} mm, tilt {r['tilt_max_deg']:.3f} deg, "
              f"self-collisions {len(r['self_collisions'])}, pass {r['gate_pass']}")
    print(f"mass {hc['mass_kg']:.4f} kg, COM x {hc['com_x_mm']:+.1f} mm, front share {hc['front_share']:.3f}")
    for leg, v in hc["legs"].items():
        print(f"  {leg}: levers hip {v['lever_hip_mm']:.1f} knee {v['lever_knee_mm']:.1f} mm | W: mg/4 {v['W_quarter_N']:.2f}, "
              f"front/rear {v['W_front_rear_N']:.2f}, sim {v['W_sim_N']:.2f} N | hip hand {v['hand_hip_Nm']:.3f} "
              f"(f/r {v['hand_fr_hip_Nm']:.3f}) sim {v['sim_hip_Nm']:.3f} | knee hand {v['hand_knee_Nm']:.3f} "
              f"(f/r {v['hand_fr_knee_Nm']:.3f}) sim {v['sim_knee_Nm']:.3f} | refined hip {v['refined_hip_Nm']:.3f} knee {v['refined_knee_Nm']:.3f} | roll sim {v['sim_roll_Nm']:.3f} N.m")
    print("wrote", OUT.relative_to(ROOT))


if __name__ == "__main__":
    main()
