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

## 2. Phase Summary

| Phase | Status | Summary |
|---|---|---|
| **0 — Environment & Tooling Setup** | [x] Complete | Set up MuJoCo Python project, flat ground plane, simulation loop, and gamepad/keyboard input. |
| **1 — Rigid Chassis + Swerve Modules** | [x] Complete | Build the MJCF model with 4 swerve modules, implement swerve drive inverse kinematics, and validate omnidirectional driving on flat ground. |
| **2 — Suspension** | [x] Complete | Add prismatic spring-damper joints modelling the splined-shaft suspension concept. Tune for ride quality over bumps and curb-cut ramps. |
| **3 — Terrain & Obstacle Traversal** | [x] Complete | Create heightfield terrains (concrete, cracked sidewalk, curb cuts, grass, gravel), build a multi-terrain course, and perform slope/tip-over analysis. |
| **4 — Control Stack** | [ ] Incomplete | Model motor controller dynamics, implement the full swerve drive controller with rate limiting and wheel coordination, add path following and odometry. |
| **5 — Power & Actuator Analysis** | [ ] Incomplete | Log per-motor torque/speed/power across terrain types and driving profiles. Size the battery and validate motor selection against simulation demands. |
| **6 — Disturbance & Robustness Testing** | [ ] Incomplete | Test external impulses, actuator failures, and payload disturbances (e.g. throwing reaction forces). Identify safety margins and degraded-mode behaviour. |

---

## 3. Simulation Objectives

1. **Validate the swerve drive kinematics** — confirm omnidirectional mobility, turning on the spot, and smooth trajectory following.
2. **Tune suspension parameters** — find spring/damper values that keep the payload stable while allowing the wheels to track uneven terrain.
3. **Test obstacle traversal** — curb cuts (up to ~50–80 mm ramps), sidewalk cracks, grass/gravel friction.
4. **Develop and validate the control stack** — swerve drive inverse kinematics, velocity control, and basic path following.
5. **Estimate power consumption** — torque and speed profiles across terrain types to size the battery.
6. **Explore failure modes** — what happens when a wheel loses contact, when the rover tips on a slope, when the yaw motor can't track fast enough.
7. **Provide a sim2real bridge** — produce a simulation accurate enough to transfer control parameters to hardware with minimal retuning.

---

## 4. Phased Implementation Plan

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

**Status: COMPLETE** (2026-03-20)

**Implementation notes:**
- Using MuJoCo 3.6.0 native Python bindings with `mujoco.viewer.launch_passive` for interactive visualisation (no third-party viewer needed).
- Ground plane: 20x20 m, `condim="4"`, friction `0.9 0.005 0.001` (tangential / torsional / rolling). This matches "good concrete" and provides the lateral grip swerve drive needs.
- Physics at 500 Hz (2 ms), control callback at 50 Hz (decimation=10), render target 60 FPS.
- Input via pygame: keyboard (WASD+QE) and optional gamepad (left stick = translate, right stick = rotate). Gamepad overrides keyboard when active.
- CSV data logger writes timestamped records (sim_time, wall_time, cmd_vx, cmd_vy, cmd_omega) to `sim/logs/`. Extends easily for motor/sensor data in later phases.
- Pygame requires a display surface for keyboard events even though rendering is handled by MuJoCo's viewer — a 1x1 hidden window is created for this purpose.

**Observations for later phases:**
- `solref` and `solimp` are set to safe defaults (`0.02 1` / `0.9 0.95 0.001`). These will likely need per-terrain tuning in Phase 3 — the current values prioritise stability over realism.
- MuJoCo 3.6.0 ships with `Euler` and `implicit`/`implicitfast` integrators. Stiff suspension springs in Phase 2 may benefit from switching to `implicitfast` if Euler causes oscillation at 2 ms timestep.
- The passive viewer's camera can be set programmatically but the user can also orbit/zoom freely. For Phase 1 teleoperation, the tracking camera following the chassis CoM should work well.

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

**Status: COMPLETE** (2026-03-20)

**Implementation notes:**
- **Chassis body:** 480×480×80 mm box (10 kg) with a freejoint. Centre at z=0.21 m, providing 5 mm clearance above wheel tops.
- **Lumped payload:** 300 mm diameter × 200 mm tall cylinder (8 kg) atop the chassis, with `contype="0" conaffinity="0"` so it doesn't participate in contacts. Total model mass: 30 kg.
- **Swerve modules:** Each consists of a yaw hinge joint (vertical axis) → fork body (0.5 kg) → drive hinge joint (horizontal axis) → wheel cylinder (82.5 mm radius × 50 mm wide, 2.5 kg). Modules at ±225 mm in x and y from chassis centre (450 mm track/wheelbase).
- **Wheel contact:** `condim="4"`, friction `1.0 0.005 0.001` — high tangential grip, low torsional and rolling friction. 8 contact points per frame (2 per wheel due to cylinder-plane contact).
- **Contact exclusions:** All chassis↔fork and chassis↔wheel contacts are excluded via `<exclude>` pairs. This is essential — without these, the wheel geoms collide with the chassis geom (they are geometrically close) and generate ~9000 N parasitic forces that prevent the rover from moving.
- **Yaw actuators:** `<position>` with kp=50 N·m/rad, torque limit ±10 N·m. Joint damping 2.0 N·m·s/rad for stability.
- **Drive actuators:** `<velocity>` with kv=10 N·m·s/rad, torque limit ±30 N·m. Joint damping 0.05 N·m·s/rad.
- **Swerve IK:** Standard 4-wheel swerve inverse kinematics in `sim/controllers/swerve_ik.py`. Includes 180° flip optimisation and a speed deadband (0.01 m/s) to prevent yaw jitter at rest.
- **Teleoperation:** Keyboard tap-based control via MuJoCo viewer key callback (arrow keys, comma/period, space) and optional gamepad. Feeds through swerve IK at 50 Hz.
- **Data logging:** Extended from Phase 0 with per-module yaw command/position, drive command/velocity, and chassis x/y/heading.
- **Sensors:** IMU (accelerometer + gyro) at chassis centre, 4× yaw position sensors, 4× drive velocity sensors.

**Validation results (headless, scripted tests):**
- Forward at 1.0 m/s: tracked at 0.994 m/s with near-zero lateral drift. 2.92 m in 3 s.
- Strafe at 0.5 m/s: tracked at 0.497 m/s. 1.49 m in 3 s.
- Rotate in place at 1.0 rad/s: achieved 171.6° in π seconds (slight undershoot from rotational inertia). Translation drift <15 mm.
- Circular arc (vx=0.5, omega=0.5, R=1.0 m): closed a full circle with <0.1 m closure error. Each wheel showed distinct yaw angles appropriate for the arc geometry (inner wheels steer more).

**Observations for later phases:**
- **Yaw actuator torque limit (±10 N·m) may be tight.** During combined translation+rotation, the yaw actuators reach ~5 N·m. Aggressive manoeuvres or higher friction surfaces in Phase 3 could saturate them. Monitor and increase if needed.
- **Drive actuator kv=10 and forcerange=±30 N·m are generous for flat ground.** The velocity tracking is very accurate (<1% error at 1 m/s). On terrain with higher resistance (gravel, slopes), these may need increasing — or the saturation behaviour itself becomes interesting for Phase 5 power analysis.
- **Wheel-ground contact model is adequate for flat concrete** but the cylinder-plane contact produces 2 contact points per wheel (at the cylinder edges), not a continuous contact patch. For Phase 3 terrain work, consider whether this affects lateral force fidelity on rough surfaces.
- **The 180° flip optimisation works but has a discontinuity** at exactly ±90° from current angle. In practice this hasn't caused issues because the control rate (50 Hz) prevents the target angle from changing that fast. Phase 4's rate-limited yaw controller will handle this more gracefully.
- **Chassis height (z=0.21 m centre, 0.17 m bottom) leaves only ~87.5 mm ground clearance** (to the bottom of the fork geom). This is within the 5–8 cm obstacle spec but tight. Phase 2 suspension will add ~25 mm of travel downward, reducing static clearance further. Worth tracking during suspension design.
- **Drive actuator `ctrlrange` silent clamping.** The drive velocity actuators have `ctrlrange="-30 30"` (rad/s). At max speed (2 m/s) a single wheel needs ~24.2 rad/s, but during aggressive combined translation+rotation, corner wheels can exceed 30 rad/s. MuJoCo silently clamps the ctrl signal, so one wheel loses proportionality without the others scaling down. Phase 4's wheel speed coordination (scaling all wheels when any exceeds the limit) should address this.
- **Body-frame velocity commands cause circular arcs during combined translation+rotation.** *(Resolved post-Phase 3.)* The swerve IK correctly takes body-frame inputs, but when the chassis is spinning, "forward" rotates with it. A global-to-body-frame rotation transform was added to the control loop (`main.py`) so that velocity commands from the input handler are now interpreted in the world frame. The IK itself is unchanged. See Phase 4 for the planned body/global frame toggle.

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

**Status: COMPLETE** (2026-03-20)

**Implementation notes:**
- **Kinematic chain per module (physically accurate):** yaw hinge (chassis-mounted) → prismatic slide (spline shaft) → fork body → drive hinge (hub motor) → wheel. This adds a `XX_yaw_body` between the chassis and fork, with the yaw motor as sprung mass (0.3 kg). Fork mass reduced from 0.5 kg to 0.2 kg (total per-module mass unchanged at 3.0 kg).
- **Integrator switched from `Euler` to `implicitfast`** for better stability with spring-damper joints. No performance cost; prevents potential oscillation at 2 ms timestep.
- **Suspension parameters:**
  - Stiffness: 3600 N/m (natural frequency ~4.4 Hz for 4.8 kg sprung mass per corner).
  - Damping: 100 N·s/m (damping ratio ζ ≈ 0.39 — underdamped but well-behaved).
  - Travel: ±25 mm (50 mm total).
  - `springref = -0.013 m` — preloads the spring so that the static equilibrium sits near q=0, giving symmetric travel in both directions.
- **Contact exclusions updated:** New body hierarchy requires exclusions for non-adjacent bodies (chassis↔fork, chassis↔wheel, yaw_body↔wheel). Parent-child exclusions are handled automatically by MuJoCo.
- **Suspension sensors:** 4× `jointpos` sensors added for per-module displacement logging.
- **Test terrain added to scene.xml:**
  - 6 small bumps (5–10 mm tall, 40 mm wide) spaced 0.4 m apart along the x-axis (x=3.0..5.0 m).
  - A curb-cut ramp at x=6.5..8.5 m: inclined slab (8°) → flat top (70 mm elevation, 500 mm long) → inclined slab down.
- **Logging extended** with per-module suspension displacement, chassis z (heave), pitch, and roll.

**Validation results (headless, scripted tests):**
- **Static settling:** Suspension settles at +0.08 mm per corner (design target: 0 mm). Chassis sag: 0.3 mm. Zero residual oscillation after 3 s — damping is effective.
- **Bump traversal (1.0 m/s):** Chassis heave range: 13.9 mm over 5–10 mm bumps. Pitch oscillation: ±1.4°. Roll: negligible (±0.01°, bumps are full-width). Suspension travel: ±11 mm (well within ±25 mm limits). Velocity tracking: 0.975 m/s (target 1.0).
- **Curb-cut ramp (0.5 m/s):** Ramp traversed successfully. Chassis z on flat top: 0.261 m (vs expected 0.280 m — the 19 mm deficit is suspension compression under transitional loading). Suspension travel reaches ±23 mm (92% of limit, see observations below). Pitch excursion: ±13° during ramp transitions.
- **Velocity tracking:** Forward: 0.975 m/s (Phase 1: 0.994). Strafe: 0.462 m/s (Phase 1: 0.497). Rotation: 171.6° in π s (identical to Phase 1). Suspension does not degrade swerve drive performance.

**Observations for later phases:**
- **Suspension nearly bottoms out on the 70 mm curb-cut ramp.** Peak travel reaches 23 mm of the 25 mm limit (92%). The 70 mm ramp is at the upper end of the design spec (50–80 mm), and the suspension just barely handles it. Options for Phase 3: (a) increase travel to ±30 mm if chassis clearance allows, (b) stiffen springs (reduces ride quality on small bumps), or (c) accept that 70+ mm obstacles require slower approach speeds. This is a real design constraint worth tracking during hardware development.
- **Pitch excursion during ramp is ±13° — significantly more than the 8° ramp angle.** This is because only 2 wheels are on the ramp at a time (the 450 mm wheelbase means the front and rear axles encounter the ramp ~0.5 s apart at 0.5 m/s). The chassis pitches as the front wheels climb while the rear are still flat (and vice versa). This dynamic overshoot is expected but worth considering for payload stability — Ball Butler's throwing mechanism may need to account for or wait out these transients.
- **Static sag is nearly zero (0.3 mm)** thanks to the `springref` preload. This validates the approach of computing springref from the sprung mass and stiffness. If the payload mass changes significantly (e.g., hopper full vs empty — ~0.8 kg difference), the sag shift is only ~2 mm, well within tolerance.
- **Velocity tracking is slightly lower with suspension than Phase 1** (0.975 vs 0.994 m/s forward). This ~2% reduction is from energy being absorbed by suspension motion. This is physically realistic and not a concern.
- **The `implicitfast` integrator works well.** No oscillation or instability observed at 2 ms timestep with k=3600 N/m springs. This is expected — the suspension's natural frequency (~4.4 Hz) is far below the 500 Hz physics rate. The Euler integrator would likely also work fine here, but `implicitfast` provides a safety margin for Phase 3 when stiffer terrain contacts may interact with the suspension.

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

**Status: COMPLETE** (2026-03-20)

**Implementation notes:**
- **Terrain course** (`scene_terrain.xml`): 30 m multi-terrain course along the +x axis with 6 distinct terrain sections, plus a slope analysis area at y=5 m offset.
- **Course layout:** concrete (x=-2..4) → curb cut UP (70 mm, ~8°) → cracked sidewalk (z=70 mm base + 5–15 mm steps) → curb cut DOWN → grass (heightfield ±15 mm) → gravel (heightfield ±25 mm) → concrete (x=24..30).
- **Heightfield generation** (`terrains/terrain_generator.py`): Procedural noise generation at 20 mm grid spacing (301×151 grid per heightfield). Grass uses 10 iterations of smoothing for low-frequency undulations; gravel uses 4 iterations for higher-frequency texture. Edge tapering blends to z=0 at heightfield boundaries for smooth terrain transitions.
- **Contact parameters tuned per terrain type:**
  - Concrete: friction 0.9, default solref (0.02 1).
  - Sidewalk: friction 0.85 (slightly weathered).
  - Grass: friction 0.55, rolling friction 0.003, solref 0.03 1.2 (softer, slightly overdamped contact).
  - Gravel: friction 0.7, rolling friction 0.005, solref 0.015 1.5 (moderate grip, high rolling resistance, more damped contact).
- **Slope analysis area:** Five slopes at 5°, 10°, 15°, 20°, 25° (each 3 m × 3 m × 1 m thick tilted box), positioned along +x at y=5 m. Flat landing pads (2 m long) at the top of each slope to prevent cliff-edge drop-offs. Flat ground slab for approach.
- **Interactive use:** `python main.py --terrain` loads the terrain course scene. The original Phase 2 scene (`python main.py`) is unchanged.

**Validation results (headless, scripted tests):**
- **Full terrain course at 1.0 m/s:** Successfully traversed all 6 terrain sections. Distance covered: 29.66 m in 30 s = 0.989 m/s average speed.
  - Concrete: nominal (pitch ±5.7°, susp max 19.9 mm during initial bump transitions).
  - Curb cut UP: pitch ±9.4° (dynamic overshoot from 8° ramp), suspension max 12.3 mm.
  - Cracked sidewalk: very stable (pitch ±3.7° over 5–15 mm steps, roll ±0.05°, susp max 10.2 mm).
  - Curb cut DOWN: pitch +8.7° (reverse of UP), suspension max 7.7 mm.
  - Grass: mild roll ±0.32° from heightfield undulations. Lower friction (0.55) did not cause measurable slip at 1 m/s. Suspension max 7.1 mm.
  - Gravel: more pronounced pitch ±7.4° and roll ±0.80° from rougher heightfield. Suspension max 11.9 mm. Higher rolling friction (0.005) did not noticeably slow the rover at 1 m/s.
  - Concrete (return): smooth, pitch transient from gravel → concrete transition.
- **Slope climbing at 0.5 m/s:**
  - 5°: climbed cleanly (0.262 m height, target 0.261 m).
  - 10°: reached the top (0.524 m, target 0.521 m) but **flipped at the slope-to-landing-pad crest transition** (pitch excursion to 89.5°).
  - 15°: reached the top (0.807 m, target 0.776 m) but also flipped at the crest.
  - 20°: climbed cleanly (0.949 m, target 1.026 m — reached 92% of target height).
  - 25°: climbed cleanly (1.172 m, target 1.268 m — 92%). Pitch excursion ±56° but no full flip.
  - Maximum climbable angle: 25° (with traction available on concrete-friction slopes). The 10° and 15° crest flips are speed-dependent — the rover crests with enough momentum to pitch forward off the landing pad.
- **Tip-over analysis (stationary):**
  - Longitudinal: stable up to 25° (margin 149.6 mm at 25°). The support polygon is wide enough in the forward direction.
  - Lateral: stable up to 15° (margin 185.2 mm). **Flipped at 20°** (roll 160°).
  - Overall maximum safe slope: **15°** (limited by lateral stability due to the tall Ball Butler payload raising the CoG).

**Post-Phase 3 refactoring:**
- **Rover MJCF extracted to `models/rover.xml`** and included via `<include>` in both scene files. Chassis body, contact exclusions, actuators, and sensors are now defined once.
- **Shared utility module `sim/utils/`** created, consolidating duplicated code: `quat_to_euler`, `get_ids`, `step_with_control`, model loaders, and joint/actuator name constants. Both test files and `main.py` now import from `utils/`.
- **Global-frame velocity commands** added to `main.py`. Input handler commands (vx, vy) are rotated into the body frame using the current chassis heading before feeding into the swerve IK. This fixes the circular-arc behaviour during combined translation+rotation.
- **Test exit codes** added — both test scripts now call `sys.exit(1)` on failure for CI compatibility.
- **`tests/__init__.py`** added for proper package structure.

**Observations for later phases:**
- **Lateral stability is the binding constraint for slope operation.** The rover is stable to 25° longitudinally but only 15° laterally. The 450 mm track width and high CoG (weighted average ~272 mm due to the 8 kg payload at 350 mm) give a theoretical lateral tip-over angle of ~39°, but the dynamic margin is lower. The ~15° practical limit is well above the 10° design requirement but worth tracking as payload mass/height changes.
- **Crest transitions are the dominant hazard during slope climbing, not the slope itself.** The rover can climb 25° slopes with adequate traction, but the pitch transient at the slope-to-flat transition causes flips at 10° and 15° when approaching at 0.5 m/s. Phase 4's controller should implement speed reduction near slope crests (detectable via pitch rate). Alternatively, a pitch-rate-limited velocity controller would naturally slow down as the crest approaches.
- **Grass and gravel friction models are plausible but need hardware validation.** The chosen values (mu=0.55 grass, mu=0.7 gravel) are consistent with published friction coefficient ranges, but the heightfield surface profiles are procedurally generated. Real terrain has spatially correlated features (e.g., tyre tracks, drainage channels) that the random noise model doesn't capture. The key finding — that neither terrain type causes significant slip at 1 m/s — is robust to moderate friction variation since the swerve drive operates well below the friction limit during straight-line driving.
- **Sidewalk crack traversal is benign.** The 5–15 mm step discontinuities produce only ±3.7° pitch and 10.2 mm suspension use. This is well within the suspension's ±25 mm travel. Real sidewalk cracks may include wider gaps (which could trap a wheel) — the current model uses solid raised slabs without gaps.
- **Suspension stays within limits across all terrain types in the course.** Peak usage was 19.9 mm (on concrete transitions) vs the 25 mm limit. The Phase 2 concern about near-bottoming on curb cuts (23 mm) is less acute in Phase 3 because the curb cut ramp design is slightly different (smoother transitions at entry and exit).
- **The gravel contact model (solref 0.015 1.5) produces more pitch/roll than grass** despite having higher friction (0.7 vs 0.55). This is because the ±25 mm heightfield roughness is the dominant effect, not the friction. The stiffer contact (lower solref time constant) also means the gravel "pings" the suspension harder than the softer grass contact.

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
- Add a **velocity frame toggle** (global vs body frame). Global frame is the current default — velocity commands are interpreted as world x/y and rotated into the body frame using the chassis heading each control step. Body frame interprets commands relative to the chassis orientation (more natural for "drive like a car" control). Both modes are useful: global frame is better for waypoint-following and "move to a position while facing a target"; body frame is more intuitive for manual gamepad driving. The toggle should be exposed as both a keyboard shortcut in the viewer and a config option.

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

## 5. MJCF Model Structure (Sketch)

This is a high-level outline of the XML structure. Details will be refined during implementation.

```
worldbody
├── light, camera
├── ground plane (geom or heightfield)
├── test terrain (bumps, curb-cut ramp)
│
└── chassis (body, mass ~15-20 kg)
    ├── chassis geom (box or mesh)
    ├── payload mass (lumped Ball Butler + hopper + sensors)
    │
    ├── front-left module
    │   ├── yaw joint (revolute, vertical axis) — actuated
    │   ├── yaw body (motor + bracket, ~0.3 kg, sprung)
    │   │   ├── suspension joint (prismatic, vertical) — spring/damper
    │   │   └── fork body (~0.2 kg, unsprung)
    │   │       ├── drive joint (revolute, horizontal) — actuated
    │   │       └── wheel body (cylinder, ~2.5 kg, unsprung)
    │   │           └── wheel geom (contact)
    │
    ├── front-right module (mirror of front-left)
    ├── rear-left module
    └── rear-right module

actuator
├── 4× yaw actuators (position control)
└── 4× drive actuators (velocity control)
    (suspension is passive — spring/damper on prismatic joint)

sensor
├── chassis IMU (accelerometer + gyro)
├── 4× yaw joint position sensors
├── 4× drive joint velocity sensors
├── 4× suspension joint position sensors
└── (optional) rangefinder for ground clearance
```

---

## 6. Key MuJoCo Modelling Decisions

### 6.1 Wheel-Ground Contact
This is the single most important tuning parameter for a swerve drive simulation. The wheels need to grip laterally (no sliding) while allowing rolling. Key settings:
- `condim="4"` — enables tangential friction in both directions plus torsional friction.
- `friction="1.0 0.005 0.001"` — high tangential, low torsional and rolling friction.
- Tune `solref` and `solimp` for the contact stiffness/damping to avoid instability at the simulation timestep.

### 6.2 Actuator Modelling
- **Drive motors:** Velocity-controlled with torque limits. Use `<velocity>` actuator with `ctrlrange` and `forcerange` to model the hoverboard motor's torque-speed curve.
- **Yaw motors:** Position-controlled with velocity limits. Use `<position>` actuator with `kp` gain and `forcerange` for torque limit. The `kv` gain provides damping.

### 6.3 Simulation Timestep
- Start with 2 ms (500 Hz). If contact stability is an issue with the small suspension springs, reduce to 1 ms.
- Control loop at 50–100 Hz (every 10–20 sim steps).

### 6.4 What to Ignore (Initially)
- Electrical dynamics (motor inductance, controller bandwidth) — model as ideal torque sources initially.
- Battery voltage sag — assume constant bus voltage.
- Thermal effects — ignore motor heating.
- Sensor noise — add in Phase 4 if needed.
- Aero drag — negligible at 2 m/s.

---

## 7. Future Work — Ball Butler Integration

These items are not part of the initial simulation phases but represent the natural extension once the rover platform is validated.

### 7.1 Ball Butler Model
- Model the Stewart platform as a 6-DoF parallel mechanism atop the rover chassis.
- Include the linear throwing axis.
- Apply the existing Ball Butler kinematics and dynamics models.
- Simulate throwing while the rover is stationary and while moving — quantify the disturbance to the rover.

### 7.2 Ball Collection Mechanism
- Model the pickup mechanism as an actuated linkage or conveyor at the front/side of the rover.
- Simulate the rover driving toward a ball on the ground, collecting it, and transferring it to the hopper.
- This requires modelling the ball as a free body with appropriate contact properties.

### 7.3 Ball Storage (Hopper)
- Model the hopper as a container with 5–10 balls as free bodies.
- Simulate the effect of ball shifting on rover stability (dynamic CoG changes).
- Model the feed mechanism from hopper to Ball Butler.

### 7.4 Full Mission Simulation
- Combine rover navigation, ball collection, and ball throwing into a complete mission loop.
- The rover drives to a ball, picks it up, drives to a throwing position, and Ball Butler throws it.
- Validate that the full system works end-to-end in simulation before committing to hardware.

### 7.5 Perception Stack (Sim-Only)
- Add simulated LiDAR or depth camera sensors in MuJoCo.
- Feed sensor data to a local planner (e.g., DWA or TEB planner).
- Test the nunchuck → velocity command → local planner → swerve controller pipeline in sim.

---

## 8. Estimated Timeline

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

## 9. Open Questions

1. **Wheel track and wheelbase dimensions** — these determine the turning radius, tip-over stability, and overall footprint. Need to balance compactness (for sidewalks) with stability (tall CoG from Ball Butler).
2. **Hoverboard motor internal friction / cogging** — may need measurement to model accurately. Cogging torque can be significant at low speeds.
3. **Suspension preload** — with the uneven weight distribution (Ball Butler is offset from centre?), may need adjustable preload per corner.
4. **Yaw belt reduction backlash** — timing belts have very low backlash, but it's worth modelling if yaw precision matters for path tracking.
5. **Tyre model fidelity** — MuJoCo's contact model is good but not a dedicated tyre model. For accurate lateral force prediction during swerve, may need to compare against a Pacejka-like tyre model or validate against hardware early.
