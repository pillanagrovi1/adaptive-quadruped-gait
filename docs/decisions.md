# Decisions

Each entry: what was decided, why, and its status.

**D1 — Project reset (24 Nov 2026).** The earlier repo benchmarked table, linear and network learners on pendulum, cart-pole and a Unitree Go1.
That was not the goal. The goal is a 12-servo quadruped (the reference design in [robot.md](robot.md)) that adapts its trot to new ground,
learning on a Cortex-M4. The old repo is archived at tag `benchmark-v1`. This repo keeps only its fixed-point Q-table, the corridor
check (C == Python, bit for bit) and the M4 tooling. Before tagging, the old repo's uncommitted D81 clean-up was committed (Rahul's choice).
**Status: done.**

**D2 — Reference design files stay out of the repo.** They are someone else's design and the licence is not confirmed.
Scripts read them from `~/reference/quadruped/`; the repo keeps only numbers measured from them. CI runs without them. **Status: applied.**

**D3 — Plate thickness 1.5 mm, not the 2 mm default.** The DXF itself says "1.5mm Aluminium", and the STEP plates are 1.5 mm thick.
A measured value beats an assumed one. **Status: applied; Rahul to confirm.**

**D4 — Printed parts weigh 0.6 x their solid mass.** Assumed (walls plus partial infill). Changes the printed parts' mass, 312 g in total, by ~5 g per 0.01. **Status: assumed.**

**D5 — Purchased parts.** Servos 65 g, 18650 cells 47 g each (both given). Electronics, bearings, screws and rod ends: STEP volume x an
assumed material density (`configs/robot.json`). Push-rods are not in the STEP: 3 mm steel rods of the measured length. Wires not counted. **Status: assumed.**

**D6 — Knee modelled as a direct hinge, driven through a tendon.** Both push-rod loops are parallelograms, so the knee servo sets the tibia's
pitch 1:1 (measured: within 0.05 deg). The knee servo is a position actuator on the fixed tendon `hip_pitch + knee`, so its torque,
damping and speed act where the real servo's do. The rods are not simulated. **Status: applied.**

**D7 — Servo travel centred on the CAD pose (hip pitch, knee servo) and on the vertical leg (hip roll).** The servos' neutral positions are not
in the CAD. The CAD's four legs are posed differently (roll 13.9-17.8 deg), so the CAD is not a reliable neutral for roll. **Status: assumed; Rahul to confirm.**

**D8 — Knee range: 10 deg away from the push-rod dead points.** At a dead point the rod lines up with the link it drives and cannot push. **Status: assumed margin.**

**D9 — Servo numbers.** Stall 3.4 N.m at 6.0 V, torque limit 0.7 x stall, 0.14 s/60 deg (7.48 rad/s), deadband 0.36 deg, back-EMF damping 0.45 N.m.s/rad
(prompt values; seller figures, not measured). Gain kp = 39 N.m/rad (full stall torque at 5 deg error) and armature 0.002 kg.m2 are assumed. **Status: assumed.**

**D10 — Command delay is a parameter: 20 ms by default, run at 10, 20, 40, 60, 80 ms.** A ring buffer in the servo wrapper (portable across MuJoCo versions).
20 ms is a guess: a measured hobby-servo quadruped had 76 ms. **Status: applied; sensitivity in Stage 2.**

**D11 — Standing height: hip pitch axis 160 mm above the floor, feet under the hips, legs vertical in roll.** Close to the CAD pose (~159 mm). **Status: chosen.**

**D12 — Contact and timing.** Foot friction 0.9 (TPU on a floor, 0.8-1.0), physics step 1 ms, control 50 Hz. **Status: assumed.**

**D13 — IMU noise and bias.** Gyro 0.005 rad/s noise, 0.01 rad/s bias; accelerometer 0.05 m/s2 noise, 0.1 m/s2 bias; a new bias per episode. **Status: assumed.**

**D14 — Simple collision shapes.** Torso box, hip-module boxes, femur and tibia capsules, sphere feet. The hip-module boxes come from the bounding
boxes of the (rotated) servos and are a little too big. Visual STL meshes (optional in the prompt) are not added yet. **Status: applied.**
