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
assumed material density (`configs/robot.json`). Push-rods are not in the STEP: 3 mm steel rods of the measured length. Wires not counted. **Status: assumed; servo mass superseded by D16 (55 g).**

**D6 — Knee modelled as a direct hinge, driven through a tendon.** Both push-rod loops are parallelograms, so the knee servo sets the tibia's
pitch 1:1 (measured: within 0.05 deg). The knee servo is a position actuator on the fixed tendon `hip_pitch + knee`, so its torque,
damping and speed act where the real servo's do. The rods are not simulated. **Status: applied.**

**D7 — Servo travel centred on the CAD pose (hip pitch, knee servo) and on the vertical leg (hip roll).** The servos' neutral positions are not
in the CAD. The CAD's four legs are posed differently (roll 13.9-17.8 deg), so the CAD is not a reliable neutral for roll. **Status: assumed; Rahul to confirm.**

**D8 — Knee range: 10 deg away from the push-rod dead points.** At a dead point the rod lines up with the link it drives and cannot push. **Status: assumed margin.**

**D9 — Servo numbers.** Stall 3.4 N.m at 6.0 V, torque limit 0.7 x stall, 0.14 s/60 deg (7.48 rad/s), deadband 0.36 deg, back-EMF damping 0.45 N.m.s/rad
(prompt values; seller figures, not measured). Gain kp = 39 N.m/rad (full stall torque at 5 deg error) and armature 0.002 kg.m2 are assumed. **Status: superseded by D16 and D17** (kept, made consistent, as the `spt5435_guess` profile).

**D10 — Command delay is a parameter: 20 ms by default, run at 10, 20, 40, 60, 80 ms.** A ring buffer in the servo wrapper (portable across MuJoCo versions). Stage 1.1: MuJoCo 4.0.1's native actuator `delay` gives the same start time (checked: 26 ms for a 20 ms delay, both), but its history starts at ctrl = 0, so the standing robot would get zero commands for the first 20 ms; the ring buffer stays.
20 ms is a guess: a measured hobby-servo quadruped had 76 ms. **Status: applied; sensitivity in Stage 2.**

**D11 — Standing height: hip pitch axis 160 mm above the floor, feet under the hips, legs vertical in roll.** Close to the CAD pose (~159 mm). **Status: chosen.**

**D12 — Contact and timing.** Foot friction 0.9 (TPU on a floor, 0.8-1.0), physics step 1 ms, control 50 Hz. **Status: assumed.**

**D13 — IMU noise and bias.** Gyro 0.005 rad/s noise, 0.01 rad/s bias; accelerometer 0.05 m/s2 noise, 0.1 m/s2 bias; a new bias per episode. **Status: assumed.**

**D14 — Simple collision shapes.** Torso box, hip-module boxes, femur and tibia capsules, sphere feet. The hip-module boxes come from the bounding
boxes of the (rotated) servos and are a little too big. Visual STL meshes (optional in the prompt) are not added yet. **Status: applied.**

**D15 — Feedback servos; the robot walks by feel (Rahul, 25 Nov 2026).** Like a person walking in the dark: no camera. The robot senses
the ground only through its legs and body: the IMU, the foot contacts and each joint's angle, speed and load. Feedback servos are allowed,
so the simulated servo is the Feetech STS3215, not the SPT5435LV PWM servo. Simulation only for now: no hardware claims.
Supersedes the "no position feedback" rule of Stage 1 (D18). The old idea of a learner that nudges step height, body lean and stride
length is dropped. **Status: decided.**

**D16 — Servo profiles; STS3215 (7.4 V) is the default.** Servos are named profiles in `configs/servos.json`; one setting in
`configs/robot.json` picks one. Default `sts3215_7v4`: other people have measured this servo (Rhoban's BAM tool, used by Open Duck
Mini v2), so its numbers are published, not seller guesses. `sts3215_12v` is derived from it (scaled to the 12 V seller figures);
`spt5435_guess` keeps the old servo for comparison. Servo mass 55 g (seller), total robot 1737.4 g. Every value and its status:
[model_card.md](model_card.md). **Status: applied; the choice of default is superseded by D19.** Supersedes D9 and the servo mass in D5.

**D17 — Setpoint rate limiter removed.** A code review found the Stage 1 servo reached only about 5.2 rad/s, not the 7.48 rad/s it
claimed. Two limits were stacked: the Python rate limiter (7.48 rad/s), and back-EMF damping 0.45 against a torque clipped to
0.7 x stall (top speed 0.7 x 3.4 / 0.45 = 5.3 rad/s). Damping plus a torque clip already set the top speed, so the limiter never
bound and the speed limit was counted twice. Now: no limiter; back-EMF is joint (or knee tendon) damping, not the actuator's `kv`
(which `forcerange` would clip). Measured after the change: each profile's step reaches its no-load speed within 5 %
([results.md](results.md)). Note: the real STS3215 firmware does rate-limit its target (BAM: 5.29 rad/s); the model's own top speed
(5.65 rad/s) is within 7 % of it, so it is left out. **Status: applied.**

**D18 — Learners may see joint feedback.** Allowed: IMU, foot contacts, joint angle, joint speed, joint load (`load_frac`), all from
`Sensors.observe()`, joint readings one control tick late. Never allowed: true orientation, height, position, terrain (`truth()`).
Rule in [architecture.md](architecture.md). Supersedes Stage 1's "joint angles are not inputs". **Status: applied.**

**D19 — Default servo profile: `sts3215_12v` (Rahul, 25 Nov 2026).** Standing with the 7.4 V servo used up to 65 % of its rated
torque (8 of 12 servos above 50 %), before any walking. The 12 V servo has twice the rated torque; standing uses at most 32 % of it.
`sts3215_7v4` stays as the **measured reference** profile. The 12 V profile is **derived, not measured** (the 7.4 V model scaled to
the seller's 12 V figures) and is labelled so everywhere. Also decided: keep the torque limit at 3.23 N·m (BAM's fitted model;
the seller's 1.91 N·m stall is only the warning threshold), keep kp 13.37 (plausible 13.4-18.8), keep the old servo shape in the
model (a real build needs redrawn mounts), and keep 55 g servo mass for every profile (stated in the model card). **Status: applied.**
