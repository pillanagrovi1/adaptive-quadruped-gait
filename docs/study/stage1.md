# Stage 1 study page: a robot in MuJoCo

```
 models/quadruped.xml (MJCF)                 what MuJoCo builds from it

 <body name="torso">                          torso ──(hip roll joint)── hip module
   <freejoint/>                                                              │
   <body name="FL_hip">                                          (hip pitch joint)
     <joint name="FL_hip_roll"/>                                             │
     <body name="FL_femur">                                                femur
       <joint name="FL_hip_pitch"/>                                          │
       <body name="FL_tibia">                                       (knee joint)
         <joint name="FL_knee"/>                                             │
         <geom name="FL_foot" type="sphere"/>                       tibia ── foot ball
```

## 1. What an MJCF file is

MJCF is MuJoCo's XML format: a text file that describes a robot as a **tree of bodies**.
- A `<body>` is one rigid part. A body inside another body hangs from it.
- A `<joint>` inside a body says how that body may move relative to its parent. A `hinge` turns about one axis.
- A `<geom>` is a shape (box, capsule, sphere) used for collisions and pictures.
- `<inertial>` gives the body's mass, centre of mass and inertia (how hard it is to spin).
- `<actuator>` is a motor. `<sensor>` is a measurement (gyro, accelerometer, touch...).

Ours is written by `scripts/make_model.py` from measured numbers. Never edit it by hand.

## 2. How a hobby servo is modelled

> **Superseded in Stage 1.1:** the servo is now the Feetech STS3215 with a published model, and the speed limit below was removed (it was counted twice). See [stage1_1.md](stage1_1.md).

```
 you send an angle ──> [ wait: delay ] ──> [ ignore tiny changes: deadband ] ──> [ move at most 7.48 rad/s ] ──> target
                                                                                                                  │
 torque = kp x (target − angle) − damping x speed,   clipped to ± 0.7 x 3.4 N.m   <──────────────────────────────┘
```

- A hobby servo is a motor with a gearbox and a small controller inside. It only knows "go to this angle".
- MuJoCo's `position` actuator does the core: **torque = kp x angle error**, clipped to the stall torque (times a safety margin 0.7).
- Our Python layer (`python/quad/servo.py`) adds what a real servo also does:
  - **delay** (the command arrives late: 20 ms by default, tested from 10 to 80 ms),
  - **deadband** (changes smaller than 0.36 deg are ignored),
  - **speed limit** (the target moves at most 60 deg per 0.14 s).
- **Damping** 0.45 N.m.s/rad stands for back-EMF: a spinning motor makes a voltage that fights its own motion.

## 3. The one bit of maths: holding torque

```
        H (hip servo) ●
                       \  femur
                        \
              knee K     ●
                        /   tibia
                       /
     foot F ●─────────   ← the floor pushes up with force W
            |<- d ->|      d = horizontal distance
```

A torque is a force times its lever: **torque = force x horizontal distance**.
When the robot stands, each foot carries about a quarter of the weight: W ≈ m g / 4 = 1.857 kg x 9.81 m/s² / 4 = 4.56 N.

- Knee servo: lever = knee → foot, horizontally, 73.8 mm. Torque ≈ 4.56 N x 0.0738 m = **0.34 N.m**.
- Hip servo: lever = hip → knee, horizontally, 73.7 mm. Torque ≈ **0.34 N.m**.

The simulation says 0.29-0.34 N.m. The rest of the difference is the front/rear weight split and the legs' own weight
([results.md](../results.md#hand-check-of-the-holding-torque)). 0.34 N.m is 10 % of the servo's 3.4 N.m stall torque: plenty of reserve for standing.

## Three questions to answer after reading

1. In `models/quadruped.xml`, which `<body>` is the parent of `FL_tibia`, and what does the `<joint>` inside `FL_tibia` let it do?
2. You command a servo to jump 30 deg. With a 20 ms delay and the 7.48 rad/s speed limit, about how long until the target reaches 30 deg? Why does the joint itself arrive a little later still?
3. If the robot carried an extra 400 g battery, centred, what would the knee servo's holding torque become (use m g / 4 x 73.8 mm)? Is that still under 25 % of stall?
