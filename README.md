# Adaptive quadruped gait

A 12-joint, four-legged robot trots in the MuJoCo simulator and adapts to uneven ground it has never seen.

**It walks by feel, like a person walking in the dark.** There is no camera. The robot senses the ground only through its legs
and body: an IMU, the four foot contacts, and each joint's real angle, speed and load, which its feedback servos (Feetech STS3215)
report. The servo model starts from numbers other researchers measured on the 7.4 V STS3215 (Rhoban's BAM, as used by Open Duck Mini v2);
the default 12 V servo is derived from it, not measured.
This is **simulation only** for now: no claims about real hardware.
The learner is a small fixed-point Q-table in C, small enough to run and learn on a Cortex-M4 microcontroller (STM32F446RE).

Earlier work: the learner benchmark this repo starts from, tag
[`benchmark-v1`](https://github.com/pillanagrovi1/Gait_adaptive_robot/tree/benchmark-v1).

**Status: Stage 1.1 done** — the robot is measured from its CAD, modelled in MuJoCo with STS3215 servo profiles, stands,
and can feel its joints (gate numbers in [`docs/results.md`](docs/results.md), the robot in [`docs/robot.md`](docs/robot.md),
every number and how sure it is in [`docs/model_card.md`](docs/model_card.md)).

![The model standing](docs/img/stage1_side.png)

## Build and test

```
bash scripts/setup_ubuntu.sh      # once: compilers, QEMU, Python environment
source .venv/bin/activate         # every new terminal
make test                         # C tests + Python tests
make -C tools/qemu_plugin         # once: QEMU instruction counter
python3 scripts/measure_m4.py smoke --check   # same learner on an emulated Cortex-M4 == laptop, bit for bit
python3 scripts/stage1_report.py              # stand test at every servo delay + torque hand check
python3 scripts/stage1_1_report.py            # Stage 1.1 gate: servo step per profile, delay test, stand test
```

Re-measuring the robot needs the reference design unzipped to `~/reference/quadruped/` (not in this repo)
and `pip install -r requirements-reference.txt`; then run `measure_reference.py`, `mass_table.py`,
`knee_linkage.py`, `make_model.py` in `scripts/`.

## Layout

```
models/     quadruped.xml (MuJoCo model) and the measured numbers it is built from
configs/    robot.json (assumed numbers) and servos.json (servo profiles), each with its source
docs/       robot.md, architecture.md, decisions.md, results.md, study/
c/          the Q-table learner (fixed point, no malloc) and its tests
m4/         Cortex-M4 firmware: benchmark build and processor-in-the-loop build
python/     quad/ (model loader, servo profiles, sensors with joint feedback, standing pose, servo step test); tinyrl/ (C binding, corridor reference)
scripts/    robot measurement, model generation, stand test, renders, M4 measurement, Ubuntu setup
tools/      QEMU plugin that counts instructions
```
