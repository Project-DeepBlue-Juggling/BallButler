# Swerve Drive Rover — MuJoCo Simulation Plan

## 1. Project Context

### 1.1 System Overview

The robot is an 8-DoF 4-wheel swerve drive rover carrying a ball-throwing robot (Ball Butler), a ball collection mechanism, and onboard ball storage. Each wheel module has two actuated axes: **drive** (wheel spin) and **yaw** (steering about the local vertical). This gives the rover full omnidirectional mobility — it can translate in any direction and rotate independently.

The rover is intended for outdoor use on sidewalks, curb cuts, grass, gravel, and smooth concrete, handling obstacles up to ~5–8 cm. It will be teleoperated via a nunchuck-style controller providing high-level velocity commands, with onboard LiDAR/depth cameras handling obstacle avoidance and path smoothing.

### 1.2 Physical Parameters (Working Estimates)

| Parameter | Estimate | Notes |
|---|---|---|
| Wheel type | Hoverboard hub motor | ~165 mm (6.5") diameter, pneumatic tire |
| Wheel mass | ~2.5 kg each | Including hub motor |
| Yaw motor | Small BLDC + belt reduction | ~5:1 ratio, ~0.3 kg motor + bracket |
| Chassis frame | Aluminium extrusion or plate | ~3–5 kg |
| Suspension travel | ~30–50 mm | Spring-damper per wheel |
| Payload (Ball Butler) | ~3–4 kg | ~350 mm diameter, ~450 mm tall |
| Ball pickup mechanism | ~2–3 kg | Mostly mechanical |
| Ball hopper (5–10 balls) | ~0.4–0.8 kg | Balls are ~70 mm, ~80 g each |
| Battery | ~3–5 kg | 36 V, 10–15 Ah Li-ion |
| Compute + sensors | ~1–2 kg | Jetson Orin + LiDAR/depth cameras |
| **All-up mass** | **~25–35 kg** | |
| Top speed | 1.5–2 m/s | Brisk walking pace |
| Wheel track / wheelbase | ~400–500 mm | TBD during chassis design |

### 1.3 Motor Controllers

| Axis | Controller | Qty | Notes |
|---|---|---|---|
| Drive (wheel spin) | moteus r4.11 | 4 | CAN-FD, 10–44 V, 100 A peak |
| Yaw (steering) | moteus c1 | 4 | CAN-FD, 10–51 V, 20 A peak |

### 1.4 Suspension Concept

The yaw motor is chassis-mounted (sprung mass). A keyed/splined shaft runs through the motor bore (or is belt-driven from the motor), supported by a linear ball spline bearing. A spring-damper acts between the chassis and the wheel fork, allowing the wheel to travel axially while the yaw motor transmits rotation through the spline. The drive motor (hoverboard hub) is unsprung, mounted directly to the wheel fork.

### 1.5 Payload — Ball Butler (Context Only)

Ball Butler is a Stewart-platform-style robot that throws and catches balls. On the rover, it would be mounted atop the chassis in a fixed position. The ball collection mechanism is a mostly-mechanical system that picks up balls from the ground and transfers them into an onboard hopper. The thrower draws from this hopper.

For simulation purposes, the Ball Butler and collection mechanism are modelled as a lumped mass in early phases and as articulated bodies in later phases.

---

## 2. Simulation Objectives

1. **Validate the swerve drive kinematics** — confirm omnidirectional mobility, turning on the spot, and smooth trajectory following.
2. **Tune suspension parameters** — find spring/damper values that keep the payload stable while allowing the wheels to track uneven terrain.
3. **Test obstacle traversal** — curb cuts (up to ~50–80 mm ramps), sidewalk cracks, grass/gravel friction.
4. **Develop and validate the control stack** — swerve drive inverse kinematics, velocity control, and basic path following.
5. **Estimate power consumption** — torque and speed profiles across terrain types to size the battery.
6. **Explore failure modes** — what happens when a wheel loses contact, when the rover tips on a slope, when the yaw motor can't track fast enough.
7. **Provide a sim2real bridge** — produce a simulation accurate enough to transfer control parameters to hardware with minimal retuning.

---

## 3. Phased Implementation Plan

### Phase 0: Environment & Tooling Setup
**Duration:** 1–2 days
**Goal:** MuJoCo environment ready, basic rendering and control loop working.

**Tasks:**
- Set up a MuJoCo Python project (using `mujoco` package directly or via `mujoco-python-viewer` for interactive visualisation).
- Create a flat ground plane with realistic friction (concrete-like, mu ≈ 0.7–1.0).
- Establish the simulation loop structure: stepped physics, control callback, data logging.
- Set up a basic gamepad/keyboard input for manual driving (this becomes the nunchuck proxy).
- Decide on simulation timestep. MuJoCo default is 2 ms; this should be fine for a rover. The control loop can run at a lower rate (e.g., 50–100 Hz) inside the simulation.

**Deliverable:** An empty scene with a ground plane and a control loop that reads input and logs data.

---

### Phase 1: Rigid Chassis + Swerve Modules (No Suspension)
**Duration:** 1–2 weeks
**Goal:** A driveable swerve drive rover on flat ground.

**Tasks:**

**1.1 — MJCF Model of the Chassis**
- Model the chassis as a single rigid body (box or simplified mesh) with appropriate mass and inertia.
- Attach 4 swerve module "mounts" at the corners. Initially, these are just fixed attachment points.

**1.2 — Swerve Module Model**
- Each module has two revolute joints:
  - **Yaw joint** — vertical axis, connects chassis to wheel fork. Actuated (position or velocity controlled).
  - **Drive joint** — horizontal axis (wheel spin axis), connects fork to wheel. Actuated (velocity controlled).
- The wheel is a cylinder or sphere-capped cylinder with appropriate radius (82.5 mm), width (~50 mm), and mass (~2.5 kg).
- MuJoCo contact: use `condim="3"` or `condim="4"` for the wheel-ground contact to get lateral friction (critical for swerve drive). Tune `friction`, `solref`, and `solimp` to approximate pneumatic rubber on concrete.

**1.3 — Swerve Drive Inverse Kinematics**
- Implement the standard swerve drive IK: given a desired chassis velocity (vx, vy, omega), compute the required wheel speed and yaw angle for each of the 4 modules.
- Handle the "180° flip" optimisation (if the target angle is >90° from current, reverse wheel direction and use the supplementary angle).
- Implement as a Python controller that runs in the simulation loop.

**1.4 — Basic Teleoperation**
- Map gamepad/keyboard input to (vx, vy, omega) commands.
- Feed through the swerve IK to actuator commands.
- Drive the rover around on flat ground. Verify omnidirectional motion, rotation, and combined translation+rotation.

**Validation criteria:**
- The rover can translate in any direction without rotating.
- The rover can rotate in place.
- The rover can follow a circular arc.
- No wheel scrub (wheels point in the correct direction during motion).

---

### Phase 2: Suspension
**Duration:** 1–2 weeks
**Goal:** Add the spring-damper suspension and validate ride quality.

**Tasks:**

**2.1 — Prismatic Joint for Suspension**
- Replace the fixed yaw-joint-to-chassis connection with a **prismatic (slide) joint** along the local vertical axis, plus the yaw revolute joint.
- Joint order (from chassis outward): prismatic (suspension travel) → revolute (yaw) → revolute (drive) → wheel body.
  - *Or*, if modelling the spline concept faithfully: revolute (yaw, chassis-mounted) → prismatic (axial slide) → fork body → revolute (drive) → wheel.
  - The second ordering is more physically accurate. The yaw motor is on the chassis and drives rotation; the spline shaft slides axially below it.
- Limit the prismatic joint range to ±25 mm (50 mm total travel) or similar.

**2.2 — Spring-Damper Tuning**
- Apply a spring + damper to the prismatic joint. MuJoCo supports this natively via joint `stiffness` and `damping` attributes, or via a custom force in the control callback.
- Starting point: for ~7 kg per corner (28 kg / 4), a natural frequency of ~3–5 Hz gives spring constant k ≈ 2500–7000 N/m. Damping ratio ~0.3–0.5 (underdamped for comfort, enough to prevent oscillation).
- Create a flat ground with small bumps (5–10 mm sine wave) and a step (50–80 mm ramp) to test.

**2.3 — Validation**
- Drive over bumps at various speeds. Log chassis pitch/roll/heave.
- Drive over a curb-cut ramp (~50–80 mm rise over ~500 mm). Verify all 4 wheels maintain contact.
- Check that suspension bottoming-out is handled (joint limits + contact).

**Validation criteria:**
- Chassis remains approximately level over small bumps.
- All wheels maintain ground contact over the curb-cut ramp.
- No excessive oscillation after a bump.

---

### Phase 3: Terrain & Obstacle Traversal
**Duration:** 1–2 weeks
**Goal:** Validate the rover on realistic outdoor terrain.

**Tasks:**

**3.1 — Terrain Models**
- Create several terrain types as MuJoCo heightfields or meshes:
  - **Smooth concrete** — flat, high friction (mu ~0.8–1.0).
  - **Cracked sidewalk** — flat with 5–15 mm step discontinuities.
  - **Curb cut** — a 50–80 mm ramp at ~8–10° slope.
  - **Grass** — slightly uneven heightfield (±10–20 mm noise), lower friction (mu ~0.5–0.7), possibly higher rolling resistance.
  - **Gravel** — rougher heightfield (±15–30 mm), moderate friction, high rolling resistance.
- For each terrain, tune MuJoCo contact parameters (friction, solref, solimp) to approximate real behaviour.

**3.2 — Multi-Terrain Course**
- Stitch terrains together into a continuous course: concrete → curb cut → sidewalk → grass → gravel → back to concrete.
- Drive the course manually and automatically (straight line, then with turns).

**3.3 — Slope and Tip-Over Analysis**
- Add slopes (5°, 10°, 15°) and drive across them (lateral and longitudinal).
- Log the centre of mass position relative to the support polygon. Identify the tip-over angle.
- This informs maximum safe operating slope and whether the CoG (with Ball Butler on top) is too high.

**Validation criteria:**
- Successful traversal of the multi-terrain course at 1 m/s.
- No tip-over on slopes ≤ 10°.
- Identified maximum safe slope angle.

---

### Phase 4: Control Stack
**Duration:** 2–3 weeks
**Goal:** Implement and tune the full control hierarchy.

**Tasks:**

**4.1 — Low-Level Motor Controllers**
- Model the moteus controllers as velocity/position loops with realistic bandwidth.
  - Drive motors: velocity control with ~50–100 Hz bandwidth.
  - Yaw motors: position control with ~20–50 Hz bandwidth (belt reduction limits speed).
- Add actuator dynamics: motor time constants, torque limits, current limits. Use MuJoCo's actuator `dyntype` and `gaintype` or implement in the control callback.

**4.2 — Swerve Drive Controller (Mid-Level)**
- Velocity command → swerve IK → individual motor commands.
- Add rate limiting on yaw angle changes to respect the physical yaw speed (~1 rev/s = 360°/s, so any angle is reachable in <0.5 s).
- Add wheel speed coordination: if one wheel can't reach its target speed (e.g., inner wheel during a tight turn), scale all wheels proportionally.

**4.3 — Path Following (High-Level)**
- Implement a simple pure-pursuit or Stanley controller that tracks a sequence of waypoints.
- The "nunchuck" input generates a velocity command; the path follower handles smoothing and obstacle avoidance at a coarse level.
- This is a placeholder for the full autonomy stack (LiDAR SLAM + local planner) but validates the control interface.

**4.4 — Odometry & State Estimation**
- Compute rover odometry from wheel encoder feedback (motor joint positions/velocities).
- Compare with ground truth from MuJoCo. Quantify drift over distance.
- Optionally add an IMU sensor in MuJoCo and fuse with wheel odometry (EKF or complementary filter).

**Validation criteria:**
- The rover follows a 10 m waypoint path with <0.2 m cross-track error on flat ground.
- Odometry drift is <5% over 50 m on flat concrete.
- The controller handles transitions between translation and rotation smoothly.

---

### Phase 5: Power & Actuator Analysis
**Duration:** 1 week
**Goal:** Extract torque/speed/power data to inform hardware sizing and battery selection.

**Tasks:**

**5.1 — Data Logging**
- Log per-motor torque, speed, and power (τ × ω) at each timestep across all terrain types.
- Log total power draw (sum of all motors) over representative driving profiles:
  - Cruising at 1.5 m/s on flat concrete.
  - Navigating a mixed terrain course.
  - Turning on the spot.
  - Climbing a curb cut.

**5.2 — Analysis**
- Compute peak and RMS torque per motor, per terrain type.
- Compute average and peak power draw per driving profile.
- Estimate battery life: (battery capacity in Wh) / (average power draw in W) = hours.
- Verify that the hoverboard motors and moteus controllers are within their continuous ratings for typical use.

**5.3 — Motor Sizing Validation**
- Compare the simulation torque/speed requirements against the hoverboard motor specs.
- Check that the yaw motor + 5:1 belt reduction provides sufficient torque for on-the-spot rotation on the highest-friction surface.

**Deliverable:** A table mapping driving scenarios to power draw, with a battery sizing recommendation.

---

### Phase 6: Disturbance & Robustness Testing
**Duration:** 1 week
**Goal:** Understand failure modes and safety margins.

**Tasks:**

**6.1 — External Disturbances**
- Apply impulse forces to the chassis (simulating a bump or push) and verify the rover recovers.
- Apply a sustained lateral force (simulating wind or a slope) and check steady-state tracking error.

**6.2 — Actuator Failures**
- Disable one drive motor and verify the rover can still move (degraded mode — 3-wheel swerve).
- Lock one yaw motor and verify the rover can still navigate (effectively a caster on that corner).
- Saturate a yaw motor (it can't keep up with commanded angle) and observe the trajectory error.

**6.3 — Payload Shift**
- Move the payload CoG off-centre (simulating uneven ball loading) and check stability.
- Add a dynamic payload disturbance (simulating Ball Butler's throwing motion — a ~0.5 kg mass moving at ~5 m/s for ~0.1 s).

**Validation criteria:**
- The rover recovers from a 50 N impulse within 1 second.
- The rover remains controllable with one failed drive motor.
- Ball Butler throwing does not cause the rover to tip or significantly deviate from its path.

---

## 4. MJCF Model Structure (Sketch)

This is a high-level outline of the XML structure. Details will be refined during implementation.

```
worldbody
├── light, camera
├── ground plane (geom or heightfield)
│
└── chassis (body, mass ~15-20 kg)
    ├── chassis geom (box or mesh)
    ├── payload mass (lumped Ball Butler + hopper + sensors)
    │
    ├── front-left module
    │   ├── yaw joint (revolute, vertical axis) — actuated
    │   ├── yaw body (motor + bracket, ~0.5 kg)
    │   │   ├── suspension joint (prismatic, vertical) — spring/damper
    │   │   └── fork body (~0.3 kg)
    │   │       ├── drive joint (revolute, horizontal) — actuated
    │   │       └── wheel body (cylinder, ~2.5 kg)
    │   │           └── wheel geom (contact)
    │
    ├── front-right module (mirror of front-left)
    ├── rear-left module
    └── rear-right module

actuator
├── 4× yaw actuators (position control)
└── 4× drive actuators (velocity control)

sensor
├── chassis IMU (accelerometer + gyro)
├── 4× yaw joint position sensors
├── 4× drive joint velocity sensors
└── (optional) rangefinder for ground clearance
```

---

## 5. Key MuJoCo Modelling Decisions

### 5.1 Wheel-Ground Contact
This is the single most important tuning parameter for a swerve drive simulation. The wheels need to grip laterally (no sliding) while allowing rolling. Key settings:
- `condim="4"` — enables tangential friction in both directions plus torsional friction.
- `friction="1.0 0.005 0.001"` — high tangential, low torsional and rolling friction.
- Tune `solref` and `solimp` for the contact stiffness/damping to avoid instability at the simulation timestep.

### 5.2 Actuator Modelling
- **Drive motors:** Velocity-controlled with torque limits. Use `<velocity>` actuator with `ctrlrange` and `forcerange` to model the hoverboard motor's torque-speed curve.
- **Yaw motors:** Position-controlled with velocity limits. Use `<position>` actuator with `kp` gain and `forcerange` for torque limit. The `kv` gain provides damping.

### 5.3 Simulation Timestep
- Start with 2 ms (500 Hz). If contact stability is an issue with the small suspension springs, reduce to 1 ms.
- Control loop at 50–100 Hz (every 10–20 sim steps).

### 5.4 What to Ignore (Initially)
- Electrical dynamics (motor inductance, controller bandwidth) — model as ideal torque sources initially.
- Battery voltage sag — assume constant bus voltage.
- Thermal effects — ignore motor heating.
- Sensor noise — add in Phase 4 if needed.
- Aero drag — negligible at 2 m/s.

---

## 6. Future Work — Ball Butler Integration

These items are not part of the initial simulation phases but represent the natural extension once the rover platform is validated.

### 6.1 Ball Butler Model
- Model the Stewart platform as a 6-DoF parallel mechanism atop the rover chassis.
- Include the linear throwing axis.
- Apply the existing Ball Butler kinematics and dynamics models.
- Simulate throwing while the rover is stationary and while moving — quantify the disturbance to the rover.

### 6.2 Ball Collection Mechanism
- Model the pickup mechanism as an actuated linkage or conveyor at the front/side of the rover.
- Simulate the rover driving toward a ball on the ground, collecting it, and transferring it to the hopper.
- This requires modelling the ball as a free body with appropriate contact properties.

### 6.3 Ball Storage (Hopper)
- Model the hopper as a container with 5–10 balls as free bodies.
- Simulate the effect of ball shifting on rover stability (dynamic CoG changes).
- Model the feed mechanism from hopper to Ball Butler.

### 6.4 Full Mission Simulation
- Combine rover navigation, ball collection, and ball throwing into a complete mission loop.
- The rover drives to a ball, picks it up, drives to a throwing position, and Ball Butler throws it.
- Validate that the full system works end-to-end in simulation before committing to hardware.

### 6.5 Perception Stack (Sim-Only)
- Add simulated LiDAR or depth camera sensors in MuJoCo.
- Feed sensor data to a local planner (e.g., DWA or TEB planner).
- Test the nunchuck → velocity command → local planner → swerve controller pipeline in sim.

---

## 7. Estimated Timeline

| Phase | Duration | Cumulative |
|---|---|---|
| Phase 0 — Environment setup | 1–2 days | 1–2 days |
| Phase 1 — Rigid chassis + swerve | 1–2 weeks | ~2 weeks |
| Phase 2 — Suspension | 1–2 weeks | ~4 weeks |
| Phase 3 — Terrain & obstacles | 1–2 weeks | ~6 weeks |
| Phase 4 — Control stack | 2–3 weeks | ~8 weeks |
| Phase 5 — Power analysis | 1 week | ~9 weeks |
| Phase 6 — Robustness testing | 1 week | ~10 weeks |

**Total estimated time to a validated simulation: ~10 weeks** (working part-time alongside other projects).

Phases 1–3 are the highest priority — they answer the fundamental question of whether the physical platform concept works. Phases 4–6 refine the design and prepare for hardware.

---

## 8. Open Questions

1. **Wheel track and wheelbase dimensions** — these determine the turning radius, tip-over stability, and overall footprint. Need to balance compactness (for sidewalks) with stability (tall CoG from Ball Butler).
2. **Hoverboard motor internal friction / cogging** — may need measurement to model accurately. Cogging torque can be significant at low speeds.
3. **Suspension preload** — with the uneven weight distribution (Ball Butler is offset from centre?), may need adjustable preload per corner.
4. **Yaw belt reduction backlash** — timing belts have very low backlash, but it's worth modelling if yaw precision matters for path tracking.
5. **Tyre model fidelity** — MuJoCo's contact model is good but not a dedicated tyre model. For accurate lateral force prediction during swerve, may need to compare against a Pacejka-like tyre model or validate against hardware early.
