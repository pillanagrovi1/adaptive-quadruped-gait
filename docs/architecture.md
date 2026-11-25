# How the pieces fit

```
          configs/robot.json  (assumed numbers)      configs/servos.json  (servo profiles, default sts3215_12v, derived)
                         |                                     |
   CAD (not in repo) --> scripts/measure_reference.py, mass_table.py, knee_linkage.py
                         |
                 models/*.json  (measured numbers)  --> scripts/make_model.py --> models/quadruped.xml
                                                                                     |
                                                          quad.model.load(profile, backlash) applies a profile
                                                                                     |
   +---------------------------------------------- python/quad ----------------------+--------------+
   |                                                                                                |
   |  controller (Stage 2+)                                                                         |
   |     |  servo angles, 50 Hz                                                                     |
   |     v                                                                                          |
   |  ServoBank (servo.py): delay -> deadband -> + neutral offset --> MuJoCo position actuators (12)|
   |                                                                |                               |
   |                                                         MuJoCo physics, 1 ms                   |
   |                                                                |                               |
   |  Sensors (sensors.py):  observe()  = gyro + accel + 4 foot contacts                --> learner |
   |                                      + 12 x (joint angle, joint speed, load_frac)              |
   |                         truth()    = true orientation, height, position            --> logs   |
   +------------------------------------------------------------------------------------------------+
```

## The rule: what a learner may see

The robot walks by feel, like a person in the dark: no camera. It has feedback servos (Feetech STS3215, which report
their angle, speed and load), an IMU and foot contacts. So:

- A learner's input comes **only** from `Sensors.observe()`. Allowed:
  - **IMU**: gyro and accelerometer (with noise and a constant bias),
  - **foot contacts** (4),
  - **joint angle** (12, quantised to 4096 steps per turn, with noise),
  - **joint speed** (12, with noise),
  - **joint load** `load_frac` (12, a fraction of stall torque, with noise; on the real servo it is a duty cycle, not a torque sensor).
  The joint readings arrive one control tick (20 ms) late. The knee servo reports the tibia's pitch, as the real one does.
- **Never allowed:** true body orientation, height, position, or anything about the terrain. These come from `Sensors.truth()`
  (and the `true_orientation` sensor) and are for logs, plots and test checks only, not even as a "helper" feature.

Any code that feeds a learner must take its observation from `observe()`. `python/tests/test_stage1_1.py` checks that no
`truth()` value appears in `observe()`; Stage 4 will add a test on the learner side.

## Servo angles vs joint angles

The model's joints are hip roll, hip pitch and knee (knee = tibia relative to femur).
The real knee servo sets the tibia's pitch (hip pitch + knee), through two parallelogram push-rods ([robot.md](robot.md)).
So commands go to the servos as servo angles; `servo.joints_to_servos` and `servos_to_joints` convert.

## The C learner

`c/` holds the fixed-point Q-table (unchanged from `benchmark-v1`, trimmed to the table). It is not connected to the robot yet (Stage 4).
