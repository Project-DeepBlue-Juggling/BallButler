# Ball Butler

An autonomous mobile robot that retrieves, stores, and throws balls to humans or other robots. Built around a 4-wheel swerve drive rover with onboard perception and a precision ball-throwing mechanism.

## Ball Butler V0 — Current State

The V0 thrower is **operational**. It accepts a target state (position and time) and executes a throw to hit that target. The mechanism was originally developed as part of the [Jugglebot](https://github.com/Project-DeepBlue-Juggling/Jugglebot) project and is now the foundation for the full Ball Butler system.

V0 is a deliberately simple, fixed-base machine — it predates the rover and is much less capable than the eventual mobile system described below. The version progression (V0 → V1+) is summarised in the comparison table at the end of this README.

**[CAD Model (Onshape)](https://cad.onshape.com/documents/b9a77ae444a199ae8fa4f0ef/w/58a21fff0c13c6549ee3697c/e/5104799fd810ebb1022a8885)**

### V0 mechanism (3 DoF, fixed-base)

- **Yaw** — unlimited rotation via a slip ring (no rewind required).
- **Pitch** — ~90° of travel: 0° = horizontal launch, 90° = vertical launch.
- **Linear axis** — a fast linear actuator provides the throwing motion itself.
- **Ball hopper** — manually loaded; a mechanical release drops one ball into the thrower each time a bumper is hit, so the thrower can reload itself once the hopper is filled.

### What works today

- Throwing to arbitrary commanded target states (position, time)
- Throw range ~2–4 m horizontal; launch height currently capped at ~50 cm by ceiling clearance during indoor testing
- Bumper-triggered single-ball release from the hopper
- Integration with Jugglebot as a ball-serving system

### What V0 does *not* do

- **No mobility** — V0 cannot move itself; it sits on a stand
- **No ball pickup** — the hopper is reloaded by hand
- **No catching** — V0 is throw-only. *No version of Ball Butler will catch balls.* The eventual rover-based system picks balls up off the ground after they land; it does not catch mid-air.

## Ball Butler V1+ — Planned System

The full Ball Butler is a standalone mobile robot — a substantial step up from V0 in every respect. V0 is essentially just the throwing subsystem on a stand; V1+ adds mobility, autonomous ball pickup, larger onboard storage, and a perception/navigation stack. V1+ comprises:

- **Rover platform** — 4-wheel swerve drive (8 DoF) for omnidirectional movement over sidewalks, curb cuts, grass, and gravel. Suspended wheels for terrain compliance.
- **Throwing mechanism** — evolved from the current thrower, mounted atop the rover.
- **Ball pickup** — a mostly-mechanical system to collect balls from the ground and feed them into onboard storage.
- **Ball storage** — onboard hopper carrying 5-10 balls (~70 mm, ~80 g each).
- **Navigation** — LiDAR and/or depth cameras for obstacle avoidance and path planning, teleoperated via a handheld controller with autonomous path smoothing.

### Key specs (targets)

| Parameter | Target |
|---|---|
| All-up mass | 25-35 kg |
| Top speed | 1.5-2 m/s |
| Obstacle clearance | Up to ~5-8 cm |
| Runtime | 1-2 hours |
| Drive motors | Hoverboard hub motors (x4) |
| Steering motors | BLDC with belt reduction (x4) |
| Motor controllers | moteus r4.11 (drive), moteus c1 (steering) |
| Compute | Jetson Orin Nano/NX |

## V0 vs V1+ at a Glance

| | **V0 (current, operational)** | **V1+ (planned)** |
|---|---|---|
| Throwing mechanism | 3 DoF (yaw / pitch / linear) | Evolved thrower atop rover (TBD architecture) |
| Yaw range | Unlimited (slip ring) | TBD |
| Pitch range | ~0–90° (horizontal → vertical) | TBD |
| Throw range | ~2–4 m horizontal | TBD |
| Catching | No | No (intentionally — no version will catch) |
| Mobility | None — fixed stand | 4-wheel swerve drive over outdoor terrain |
| Ball pickup | Manual (hand-loaded hopper) | Autonomous ground pickup |
| Hopper capacity | Small, bumper-triggered single-ball release | 5–10 balls |
| Perception / nav | None | LiDAR / depth cameras, teleop with autonomy |

## Development Approach

The rover platform is being developed **simulation-first** in MuJoCo before committing to hardware. The full simulation plan is documented in [`BB_rover_mujoco_plan.md`](BB_rover_mujoco_plan.md) and covers six phases:

1. ~~**Environment & tooling setup**~~ — MuJoCo scene, control loop, input handling **(done)**
2. ~~**Rigid chassis + swerve modules**~~ — driveable swerve drive on flat ground, inverse kinematics, teleoperation **(done)**
3. ~~**Suspension**~~ — spring-damper prismatic joints modelling the splined-shaft suspension concept, ride quality tuning **(done)**
4. ~~**Terrain & obstacle traversal**~~ — heightfield terrains (concrete, cracked sidewalk, curb cuts, grass, gravel), multi-terrain course, slope/tip-over analysis **(done)**
5. **Control stack** — motor controller modelling, swerve drive controller, velocity frame toggle (global/body), path following, odometry & state estimation
6. **Power & actuator analysis** — torque/speed/power logging, battery sizing, motor sizing validation
7. **Disturbance & robustness testing** — external forces, actuator failures, payload shift from throwing

The simulation will later extend to include Ball Butler integration (throwing disturbance on the rover), ball collection, hopper dynamics, and a simulated perception stack.

## Repository Structure

```
BallButler/
├── ball_butler_main/               # Thrower firmware (Arduino/Teensy)
│   ├── ball_butler_main.ino        #   Main sketch — state machine orchestration
│   ├── StateMachine.cpp/.h         #   State machine (BOOT → IDLE → THROWING → RELOADING)
│   ├── CanInterface.cpp/.h         #   CAN-FD communication with ODrives
│   ├── PitchAxis.cpp/.h            #   Pitch axis ODrive position control
│   ├── YawAxis.cpp/.h              #   Yaw axis brushed DC motor control
│   ├── HandPathPlanner.cpp/.h      #   Throw trajectory planning
│   ├── HandTrajectoryStreamer.h     #   Trajectory streaming to actuators
│   ├── Proprioception.cpp/.h       #   Central axis state repository
│   ├── Trajectory.h / TrajFrame.h  #   Trajectory and frame data structures
│   ├── RobotState.h                #   Robot state definitions
│   ├── BallButlerConfig.h          #   Thrower configuration parameters
│   ├── hardware_config.h           #   Auto-generated hardware constants (from Jugglebot config)
│   └── protocol_config.h           #   CAN protocol configuration
│
├── ball_butler_circuit_diagram/    # KiCad project — main electronics schematic
│   ├── *.kicad_sch                 #   Schematic sheets (main + Power and CAN)
│   ├── *.kicad_pcb                 #   PCB layout
│   ├── Ball_Butler_*.kicad_sym     #   Custom symbol libraries
│   └── Ball Butler Schematic*.pdf  #   Exported schematic PDFs
│
├── hand_sensor_pcb_rail/           # KiCad project — hand sensor PCB rail
│   ├── *.kicad_sch / *.kicad_pcb   #   Schematic and PCB layout
│   └── Production/                 #   Gerber files for manufacturing
│
├── zTesting/                       # Test scripts and experiments
│   ├── ball_butler_spacemouse.py   #   SpaceMouse teleoperation
│   ├── simulating_throw_types.py   #   Throw trajectory simulation
│   ├── yaw_inverse_kinematics.py   #   Yaw axis IK exploration
│   ├── calibrating_yaw_axis_with_mocap/  # MoCap-based yaw calibration
│   ├── sensored_hand_testing/      #   Hand sensor testing
│   ├── throw_testing/              #   Throw characterisation
│   └── ...                         #   Other hardware tests
│
├── BB_rover_mujoco_plan.md         # MuJoCo simulation plan for the rover platform
├── sim/                            # MuJoCo rover simulation (Phase 3 complete)
│   ├── main.py                    #   Simulation entry point — control loop, global-frame velocity, terrain support
│   ├── config.py                  #   Simulation parameters
│   ├── input_handler.py           #   Keyboard/gamepad → velocity commands
│   ├── data_logger.py             #   CSV data logging
│   ├── models/                    #   MJCF model files
│   │   ├── rover.xml              #   Shared rover body, actuators, sensors, contacts (included by scenes)
│   │   ├── scene.xml              #   Flat ground + Phase 2 test terrain (bumps, curb-cut ramp)
│   │   └── scene_terrain.xml      #   Multi-terrain course + slope analysis area (Phase 3)
│   ├── controllers/               #   Control algorithms
│   │   └── swerve_ik.py           #   Swerve drive inverse kinematics (4-wheel, with 180° flip optimisation)
│   ├── utils/                     #   Shared utilities (transforms, model helpers, sim runner, constants)
│   ├── terrains/                  #   Heightfield terrain generation
│   │   └── terrain_generator.py   #   Procedural grass/gravel heightfield generation
│   ├── tests/                     #   Simulation test scenarios (exit 1 on failure)
│   │   ├── test_suspension.py     #   Phase 2 headless suspension validation
│   │   └── test_terrain.py        #   Phase 3 terrain course + slope/tip-over validation
│   ├── logs/                      #   CSV data logs (gitignored)
│   └── requirements.txt           #   Python dependencies
│
├── bb_pitch_odrive_micro_config.json  # ODrive configuration for the pitch axis
└── .gitignore
```

### Planned additions

As the rover and full system are built out, expect these directories to appear:

```
BallButler/
├── rover/                          # Rover platform (planned)
│   ├── firmware/                   #   Motor controller firmware (moteus)
│   ├── config/                     #   Rover hardware configuration
│   └── cad/                        #   Mechanical design exports
│
├── navigation/                     # Navigation & perception (planned)
│   ├── local_planner/              #   Obstacle avoidance
│   └── odometry/                   #   Wheel odometry + IMU fusion
│
└── config/                         # Shared configuration (planned)
```

## Related Projects

- **[Jugglebot](https://github.com/Project-DeepBlue-Juggling/Jugglebot)** — the robot that Ball Butler (v0) was originally built to serve. Shared configuration files (e.g. `hardware_config.h`) are generated by Jugglebot and consumed by this repo.
