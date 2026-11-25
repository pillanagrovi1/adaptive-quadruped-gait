# Results

## Stage 1: the robot stands (24 Nov 2026)

![The model in its standing pose, seen roughly from where photo 20201107_124135.jpg was taken](img/stage1_side.png)

Test: the standing pose (hips 160 mm up, feet under the hips) held for 10 s on flat ground, through the full servo model.
Reproduce: `python3 scripts/stage1_report.py` (writes `results/stage1_stand.json`). CI runs the same test at every delay.

**Gate: passed at every command delay (10, 20, 40, 60, 80 ms).** The numbers below are identical at all five delays:
the command never changes while standing, so the delay has nothing to delay. Delay matters once the robot moves (Stage 2).

| Check | Result | Gate |
|---|---|---|
| Highest servo torque | 12.6 % of stall (FR knee servo, during the first second) | under 50 % ✅ |
| Highest torque after the first second | 10.0 % of stall (FR knee servo) | under 25 % sustained guide ✅ (no joint flagged) |
| Body height | starts at 160.0 mm, sinks 1.9 mm in the first second, then drifts 0.000 mm | — |
| Body tilt | at most 0.044 deg | — |
| Self-collision | none | none ✅ |
| Anything but the feet touching the floor | none | none ✅ |

The 1.9 mm sink is the servos' proportional gain: they need a small angle error (about 0.5 deg) to produce the holding torque.

Per servo (N.m, and % of the 3.4 N.m stall; "after 1 s" = steady holding):

| Leg | hip roll | hip pitch | knee servo | hip pitch after 1 s | knee servo after 1 s |
|---|---|---|---|---|---|
| FL | 0.146 (4.3 %) | 0.389 (11.4 %) | 0.422 (12.4 %) | 9.1 % | 9.9 % |
| FR | 0.145 (4.3 %) | 0.393 (11.6 %) | 0.428 (12.6 %) | 9.2 % | 10.0 % |
| RL | 0.136 (4.0 %) | 0.357 (10.5 %) | 0.396 (11.6 %) | 8.6 % | 9.4 % |
| RR | 0.135 (4.0 %) | 0.364 (10.7 %) | 0.403 (11.8 %) | 8.6 % | 9.4 % |

## Hand check of the holding torque

The foot pushes up with force W. The knee servo turns the tibia about the knee; the hip servo turns the femur about the hip
(the tibia moves with it without turning, because the push-rods hold its angle). So:

- knee servo torque ≈ W x (horizontal distance knee → foot) = W x 73.8 mm
- hip servo torque ≈ W x (horizontal distance hip → knee) = W x 73.7 mm

| Leg | W = m g / 4 | W, front/rear split | quick estimate (front/rear W) | + legs' own weight | simulated (after 3 s) |
|---|---|---|---|---|---|
| FL hip / knee | 4.56 N | 4.69 N | 0.345 / 0.346 N.m | 0.318 / 0.337 | **0.310 / 0.337** |
| FR hip / knee | 4.56 N | 4.69 N | 0.345 / 0.346 | 0.319 / 0.338 | **0.312 / 0.338** |
| RL hip / knee | 4.56 N | 4.43 N | 0.326 / 0.327 | 0.300 / 0.319 | **0.292 / 0.319** |
| RR hip / knee | 4.56 N | 4.43 N | 0.326 / 0.327 | 0.301 / 0.320 | **0.294 / 0.320** |

(m g / 4 alone gives 0.336 N.m for every joint.)

Why the quick estimate is off, and by how much:
1. **Front/rear split** (±4 %): the centre of mass is 7.2 mm behind the body centre, but the front hips are 20 mm closer to it than the rear hips. The front feet carry 51.4 %.
2. **The legs' own weight** (hip −8 %, knee −2.5 %): the foot force W includes the weight of the femur (42 g) and tibia (27 g), which pull the other way on shorter levers. The knee only feels the tibia; the hip feels both.
3. With both corrections the **knee matches the simulation exactly** (to 0.001 N.m). The **hip is still 0.008 N.m (2.5 %) above** the simulation. The feet have no sideways force (checked: 0.000 N), so that is not it. **Not explained yet.**

The hip roll servos hold 0.135-0.146 N.m: the feet stand 39 mm outboard of the roll axes. No hand check was asked for these.

## Knee push-rods

Tibia angle vs knee-servo angle: 1:1 within 0.05 deg over the usable range (−83 to +75 deg from the CAD pose). Details in [robot.md](robot.md#the-knee-push-rods).

## Cortex-M4 (from the trimmed C core)

`python3 scripts/measure_m4.py smoke --check`: the Q-table learns bit-identically on the emulated M4 and the laptop for all 3 test configurations.
These are QEMU instruction counts, not timing on a real chip.

| Config | Peak RAM (B) | Instructions per learning step |
|---|---|---|
| corridor table (6 cells x 2 actions) | 516 | 371.6 |
| corridor table + Q(lambda) traces | 1032 | 1068.8 |
| 4 inputs x 6 bins x 7 actions (a size Stage 4 might use; a guess) | 36776 | 801.1 |
