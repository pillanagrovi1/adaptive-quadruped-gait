# Stage 1.1 study page: a servo that can feel

```
  a FEEDBACK SERVO (Feetech STS3215) is four things in one box

    bus command ──> ┌────────────────────┐   voltage   ┌───────┐   ┌──────────────────┐
   "go to 60°"      │ its own controller │ ──────────> │ motor │ ─>│ gears 345:1      │──> output shaft ──> leg
                    │  (a P controller)  │             └───────┘   └──────────────────┘          │
                    └────────────────────┘                                                         │
                          ▲        │                                                               │
                          │        └──> bus reply: angle, speed, load  ──> the robot "feels"      │
                          │                                                                        │
                          └──────────────── encoder (magnet + sensor, 4096 steps per turn) <───────┘
```

## 1. What a feedback servo is

- **Motor**: a small DC motor. More voltage = more torque.
- **Gears**: 345 turns of the motor make 1 turn of the shaft. Slower, but 345 x stronger.
- **Encoder**: a magnet on the shaft and a sensor that reads its angle, 4096 steps per turn (0.088° per step).
- **Controller**: compares "where I should be" with "where I am" and drives the motor. It also **answers questions on the bus**: "what is your angle, speed, load?"

The old hobby servo (SPT5435LV) had all of this inside too, but it could not answer. The robot only knew what it had commanded.

## 2. What the robot can feel now

`Sensors.observe()` gives, every 20 ms control tick:

| Signal | Shape | Unit | Note |
|---|---|---|---|
| gyro, accel | 3, 3 | rad/s, m/s² | IMU, with noise and bias |
| contact | 4 | yes/no | one per foot |
| joint_angle | 12 | rad | quantised to 4096 steps, noise; the knee servo reports the tibia's pitch |
| joint_speed | 12 | rad/s | noise |
| load_frac | 12 | fraction of stall | how hard the servo is working; on the real servo a duty cycle, **not** a torque sensor |

The joint readings are one tick (20 ms) old: reading 12 servos over the bus takes time.

**How "commanded minus real" shows the leg pushing on something.** The servo's controller makes torque ≈ kp x (commanded − real angle).

```
   swing leg in the air:  commanded 30°, real 29.9°  → tiny error   → tiny load   ("nothing in the way")
   foot hits a stone:     commanded 30°, real 25.0°  → 5° error     → big load    ("something is pushing back")
```

So a big gap between what you asked for and what the encoder reports, together with a high `load_frac`, means the leg is
pressing on something. That is how the robot can feel the ground in the dark.

## 3. The one bit of maths: why damping plus a torque clip sets the top speed

The motor's back-EMF acts like a brake that grows with speed: brake torque = **damping x speed**.
The servo can push with at most **forcerange** (the torque clip). The joint speeds up until the brake eats all of it:

```
   push   = forcerange                     (clipped: the error is big while it is far from the target)
   brake  = damping x speed
   top speed: push = brake   →   top speed ≈ forcerange / damping
```

(Gear friction takes a little more: top speed ≈ (forcerange − frictionloss) / damping.)

- STS3215 7.4 V: (3.23 − 0.068) / 0.56 = **5.65 rad/s**. The seller says 5.45 rad/s, and the step test measures 5.53 rad/s.
- The Stage 1 bug: torque clip 0.7 x 3.4 = 2.38 N·m, damping 0.45 → top speed 2.38 / 0.45 = **5.3 rad/s**.
  A Python rate limiter was also set to 7.48 rad/s. The physics already stopped the servo at 5.3, so the limiter never
  did anything, and the servo was 30 % slower than its spec. One speed limit, not two (decision D17).

## Three questions to answer after reading

1. The FL knee servo reads `joint_angle` = 0.40 rad and the hip pitch servo reads 0.70 rad. What is the knee *joint* angle (tibia relative to femur)? Use `servos_to_joints`.
2. A swing leg is commanded to 0.50 rad. Its `joint_angle` says 0.42 rad and its `load_frac` is 0.6, while the other swing leg shows 0.49 rad and 0.05. What is probably happening to the first leg? Why should you remember that the reading is 20 ms old?
3. The 12 V profile has forcerange 4.97 N·m and damping 0.996 N·m·s/rad (friction 0.068 N·m). What top speed do you expect? The step test measured 4.92 rad/s. Does your answer match?
