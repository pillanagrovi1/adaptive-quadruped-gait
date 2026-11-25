# How the pieces fit

```
                 configs/robot.json  (every assumed number, with its source)
                         |
   CAD (not in repo) --> scripts/measure_reference.py, mass_table.py, knee_linkage.py
                         |
                 models/*.json  (measured numbers)  --> scripts/make_model.py --> models/quadruped.xml
                                                                                     |
   +---------------------------------------------- python/quad ----------------------+--------------+
   |                                                                                                |
   |  controller (Stage 2+)                                                                         |
   |     |  servo angles, 50 Hz                                                                     |
   |     v                                                                                          |
   |  ServoBank (servo.py): delay -> deadband -> rate limit --> MuJoCo position actuators (12)      |
   |                                                                |                               |
   |                                                         MuJoCo physics, 1 ms                   |
   |                                                                |                               |
   |  Sensors (sensors.py):  observe()  = gyro + accel (noise, bias) + 4 foot contacts  --> learner |
   |                         truth()    = true orientation, height                       --> logs   |
   +------------------------------------------------------------------------------------------------+
```

## The rule: what a learner may see

The real robot has hobby servos (no position feedback), an IMU and foot switches. So:

- A learner's input comes **only** from `Sensors.observe()`: gyro, accelerometer (both with noise and a constant bias) and the four foot contacts.
- `Sensors.truth()` (the `true_orientation` framequat sensor, body height) is for logs, plots and test checks. **It is never part of what a learner sees**, not even as a "helper" feature.
- Joint angles are not inputs either: the robot cannot measure them. The controller knows only what it commanded.

Any code that feeds a learner must take its observation from `observe()`. Stage 4 will add a test for this.

## Servo angles vs joint angles

The model's joints are hip roll, hip pitch and knee (knee = tibia relative to femur).
The real knee servo sets the tibia's pitch (hip pitch + knee), through two parallelogram push-rods ([robot.md](robot.md)).
So commands go to the servos as servo angles; `servo.joints_to_servos` and `servos_to_joints` convert.

## The C learner

`c/` holds the fixed-point Q-table (unchanged from `benchmark-v1`, trimmed to the table). It is not connected to the robot yet (Stage 4).
