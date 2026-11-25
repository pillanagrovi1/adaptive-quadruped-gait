# Model card: every number the simulation uses

```
   where each number comes from                       how sure we are
   ─────────────────────────────                      ───────────────
   CAD of the reference design ──> scripts ──┐        measured   = measured in this repo (from the CAD)
   Open Duck Mini v2 / BAM files ────────────┤        published  = someone else measured or specified it (link)
   seller spec pages ────────────────────────┼──>  configs/*.json ──> models/quadruped.xml ──> simulation
   a formula on the numbers above ───────────┤        derived    = computed from other numbers (formula shown)
   a guess ──────────────────────────────────┘        assumed    = a guess, nobody measured it
```

**How to read this:** if a result depends a lot on an **assumed** number, treat the result as a guess too.
Sources used below:
- **OD** = Open Duck Mini v2 MJCF, [open_duck_mini_v2.xml @ f698836](https://github.com/apirrone/Open_Duck_Playground/blob/f6988368da959156d94fcd53b76864724dc52860/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml), default class `sts3215` (lines 45-55)
- **BAM** = Rhoban BAM, [params/feetech_sts3215_7_4V/m1.json @ 279c482](https://github.com/Rhoban/bam/blob/279c4828c466e7e19b631c45203b0e3582be9869/bam/params/feetech_sts3215_7_4V/m1.json)
- **Seller** = [openelab STS3215-C018 page](https://openelab.io/products/feetech-sts3215-c018-servo-12v): C018 spec table, and its "Product Comparison" table for the C001 (7.4 V)

## The table

| Number | Value | Unit | Status | Source or formula |
|---|---|---|---|---|
| **Servo, `sts3215_7v4`** (Feetech STS3215-C001, 7.4 V): the **measured reference** profile | | | | `configs/servos.json` |
| position gain kp | 13.37 (plausible 13.4-18.8) | N·m/rad | published | OD `<position kp="13.37">` (a commented-out line has 17.12; BAM m1 gives 18.8). With 13.37 the 60° step overshoots 4.6° |
| torque limit (forcerange) | ±3.23 (plausible 1.9-3.4) | N·m | published | OD; measured by BAM's fitted model. Low end: seller stall 1.91; high end: BAM m1 3.41 |
| back-EMF damping | 0.56 | N·m·s/rad | published | OD; = BAM kt²/R = 1.178² / 2.479 = 0.559 |
| gear friction (frictionloss) | 0.068 | N·m | published | OD |
| armature (motor inertia at the output) | 0.027 | kg·m² | published | OD |
| backlash (option, off by default) | ±0.5 | deg | published | OD class `backlash`; seller "gear backlash ≤ 0.5°" |
| backlash hinge damping / armature | 0.01 / 0.01 | N·m·s/rad / kg·m² | published | OD class `backlash` |
| stall torque | 1.912 | N·m | published | seller C001: 19.5 kg·cm at 7.4 V. Used only as the warning threshold (gates, `load_frac`), not as the torque limit |
| rated (continuous) torque | 0.490 | N·m | published | seller C001: 5 kg·cm at 7.4 V |
| no-load speed | 5.454 | rad/s | published | seller C001: 0.192 s/60° → 1.0472 / 0.192 |
| top speed the model reaches | 5.646 | rad/s | derived | (forcerange − frictionloss) / damping = (3.23 − 0.068) / 0.56 |
| deadband | 0.088 | deg | assumed | one encoder step (360/4096); the firmware dead-zone setting is not checked |
| servo mass | 55 | g | published | seller: 55 ± 1 g |
| **Servo, `sts3215_12v`** (STS3215-C018, 12 V): the **default profile** (D19), **derived, not measured** | | | | s_τ = 30 / 19.5 = 1.5385, s_ω = 0.192 / 0.222 = 0.8649 |
| kp | 20.57 (plausible 20.6-28.9) | N·m/rad | derived | 13.37 × s_τ (range: 13.4-18.8 × s_τ). The 60° step overshoots 1.2° |
| forcerange | ±4.969 (plausible 2.9-5.2) | N·m | derived | 3.23 × s_τ (range: 1.91-3.41 × s_τ = 2.94-5.25) |
| damping | 0.996 | N·m·s/rad | derived | 0.56 × s_τ / s_ω (top speed scales by s_ω) |
| frictionloss, armature | 0.068, 0.027 | N·m, kg·m² | assumed | same as 7.4 V (same 345:1 gearbox) |
| stall / rated torque | 2.942 / 0.981 | N·m | published | seller C018: 30 / 10 kg·cm at 12 V. Stall is the warning threshold |
| no-load speed | 4.717 | rad/s | published | seller C018: 0.222 s/60° |
| mass, backlash | 55 g, ±0.5° | | published | seller |
| **Servo, `spt5435_guess`** (old PWM servo, comparison only) | | | | |
| stall torque = forcerange | 3.4 | N·m | assumed | seller spec at 6.0 V, never checked; the old 0.7 margin is dropped (D17) |
| no-load speed | 7.48 | rad/s | derived | seller 0.14 s/60° → 1.0472 / 0.14 |
| damping | 0.4545 | N·m·s/rad | derived | stall / no-load speed = 3.4 / 7.48 |
| kp | 39 | N·m/rad | assumed | full stall at 5° error |
| armature | 0.002 | kg·m² | assumed | |
| frictionloss | 0 | N·m | assumed | unknown |
| rated torque | 0.85 | N·m | assumed | 25 % of stall (no seller rating) |
| deadband | 0.36 | deg | derived | 4 µs of the 500-2500 µs range × 180° / 2000 µs |
| mass | 65 | g | assumed | sellers say 62-72 g. **Not used:** the model's servos weigh 55 g with every profile (known limits) |
| **Servo, all profiles** | | | | `configs/robot.json` |
| profile in use | `sts3215_12v` (derived) | | chosen | D19 (Rahul); `sts3215_7v4` is the measured reference |
| servo travel | 180 | deg | assumed | reference design's servo; the STS3215 turns 360°, the linkage limits it |
| neutral offset per servo | 0 (option ±0.02) | rad | assumed | encoder servos are calibrated on hardware; ±0.02 rad for robustness tests later |
| command delay | 20 (runs at 10, 20, 40, 60, 80) | ms | assumed | a measured hobby-servo quadruped had 76 ms (Weddington et al., arXiv 2607.26434) |
| **Joint feedback (what the robot feels)** | | | | `configs/robot.json` → `joint_sensors` |
| angle resolution | 4096 steps/turn (0.088°) | | published | seller: 12-bit magnetic encoder |
| angle noise | 0.0015 | rad | assumed | about one encoder step |
| speed noise | 0.05 | rad/s | assumed | |
| load noise | 0.02 | of stall | assumed | |
| load_frac | actuator force / stall torque | – | derived | stand-in for the real servo's "present load", which is a drive duty cycle, not a torque |
| read delay | 1 tick (20 ms) | | assumed | a bus read of 12 servos takes about 12 × (8 + 15 bytes) × 10 bits / 1 Mbit/s ≈ 2.8 ms (calculated, not measured), so one tick is safe |
| **IMU and contacts** | | | | |
| gyro noise / bias | 0.005 / 0.01 | rad/s | assumed | MPU-6050 class |
| accel noise / bias | 0.05 / 0.1 | m/s² | assumed | |
| foot contact threshold | 0.5 | N | assumed | `Sensors(contact_N=0.5)` |
| **Contact and timing** | | | | |
| foot friction | 0.9 | – | assumed | TPU on a floor, 0.8-1.0 |
| physics step | 1 | ms | assumed | |
| control rate | 50 | Hz | assumed | |
| standing hip height | 160 | mm | chosen | close to the CAD pose (~159 mm) |
| knee margin from push-rod dead points | 10 | deg | assumed | |
| **Geometry** | | | | [robot.md](robot.md#model-numbers) |
| link lengths, hip positions, foot radius, body box | see robot.md | mm | measured | STEP (CAD) |
| knee push-rods 1:1 | within 0.05 | deg | measured | `scripts/knee_linkage.py` |
| femur / tibia collision capsule radius | 10 / 7 | mm | assumed | fits inside the STLs (`make_model.py`) |
| **Mass** (total 1737.4 g) | | | | [robot.md](robot.md#masses) |
| printed part volumes, plate areas, screw / bearing volumes | per part | cm³, mm² | measured | STL, DXF, STEP |
| printed infill factor | 0.6 | – | assumed | walls + 20-30 % infill; ±0.01 changes the mass by ~5 g |
| PLA / TPU density | 1.24 / 1.21 | g/cm³ | assumed | |
| aluminium / steel / POM / nylon density | 2.70 / 7.85 / 1.41 / 1.15 | g/cm³ | assumed | |
| plate thickness | 1.5 | mm | measured | note in the DXF |
| electronics (Orange Pi, DC-DC, fan, meter, switch) | STEP volume × 1.2-2.7 | g/cm³ | assumed | densities in `configs/robot.json` |
| screws | STEP volume × steel | | measured volume, assumed density | |
| 18650 cell | 47 | g | assumed | |
| push-rods | 3 mm steel, measured length | | assumed diameter | not in the STEP |
| wires and connectors | 0 | g | assumed | not in the CAD, not counted |

## Known limits

- **Collision shapes are simple** (boxes, capsules, spheres). The hip boxes are a little too big. Not changed in this stage.
- **Knee push-rods:** the 1:1 tendon is checked only away from the dead points (10° margin). Near a dead point the real linkage stops being 1:1.
- **Bellcrank and long push-rod masses are lumped onto the femur**, but they turn with the tibia's pitch, so on the real robot their weight loads the knee servo, not the hip servo. About 13 g per leg (bellcrank 3.3 g, spacer 1.5 g, long rod 5.5 g, its two rod ends 2.5 g; `models/mass_measured.json`), against a femur body of 41.6 g.
- **The CAD is drawn for the SPT5435LV.** The STS3215 is 45.2 × 24.7 × 35 mm (seller). Shapes and positions are still the SPT5435LV's; only the mass is the STS3215's. **A real STS3215 build needs redrawn mounts.**
- **Servo mass is 55 g with every profile (a simplification).** The mass table is built once, with the default profile's 55 g. Both STS3215 profiles are 55 g, so for them it is exact. Only the comparison profile `spt5435_guess` (65 g in `configs/servos.json`) runs 10 g per servo, 120 g in total, too light.
- **No firmware speed limit.** BAM's STS3215 model also rate-limits the target inside the servo (identified max_velocity 5.29 rad/s, BAM m1). Here damping + the torque clip set the top speed (5.65 rad/s in the model, seller 5.45 rad/s). That is within 7 % of BAM's limit, so the limiter is left out (D17).
- **Open Duck and BAM m1 differ a little** (see the next table). The profile uses Open Duck's values, because they are what a walking robot uses.
- **Stall torque vs torque limit:** the seller's stall torque (1.91 N·m at 7.4 V) is lower than the model's torque limit (3.23 N·m, BAM's fitted model). The model keeps 3.23; the seller's stall is used only as the warning threshold (the stricter choice).
- **The STS3215 step overshoots** by 4.6° at 7.4 V and 1.2° at 12 V in the model (Open Duck's kp with this armature). Not checked against a real servo.
- **The default profile is derived, not measured.** Every 12 V number is the 7.4 V model scaled by two seller ratios. Nobody has measured a 12 V STS3215 for this model.

### Open Duck vs BAM m1 (STS3215, 7.4 V)

BAM m1 converted to MuJoCo terms with BAM's own constants (vin 7.4 V, firmware kp 32, error_gain 0.166, max_pwm 0.97; `bam/feetech/actuator.py`):

| | Open Duck (used) | BAM m1, derived | formula |
|---|---|---|---|
| kp (N·m/rad) | 13.37 (17.12 commented out) | 18.77 | kt/R × vin × kp_fw × error_gain × error_gain_ratio |
| forcerange (N·m) | 3.23 | 3.41 | kt/R × vin × max_pwm |
| damping (N·m·s/rad) | 0.56 | 0.559 (+ 0.060 viscous) | kt²/R (+ friction_viscous) |
| frictionloss (N·m) | 0.068 | 0.052 | friction_base |
| armature (kg·m²) | 0.027 | 0.026 | armature |
| speed limit in firmware (rad/s) | none | 5.29 | max_velocity |
