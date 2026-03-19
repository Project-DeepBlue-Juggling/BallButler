# Ball Butler

An autonomous mobile robot that retrieves, stores, and throws balls to humans or other robots. Built around a 4-wheel swerve drive rover with onboard perception and a precision ball-throwing mechanism.

## Current State

The throwing subsystem is **operational**. It accepts a target state (position and time) and executes a throw to hit that target. This mechanism was originally developed as part of the [Jugglebot](https://github.com/Project-DeepBlue-Juggling/Jugglebot) project and is now the foundation for the full Ball Butler system.

**[CAD Model (Onshape)](https://cad.onshape.com/documents/b9a77ae444a199ae8fa4f0ef/w/58a21fff0c13c6549ee3697c/e/5104799fd810ebb1022a8885)**

### What works today

- Throwing to arbitrary commanded target states (position, time)
- Ball hopper with manual reloading (of the hopper - BB can automatically reload itself)
- Integration with Jugglebot as a ball-serving system

## Planned System

The full Ball Butler is a standalone mobile robot comprising:

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

## Development Approach

The rover platform is being developed **simulation-first** in MuJoCo before committing to hardware. The full simulation plan is documented in [`BB_rover_mujoco_plan.md`](BB_rover_mujoco_plan.md) and covers six phases:

1. **Environment & tooling setup** — MuJoCo scene, control loop, input handling
2. **Rigid chassis + swerve modules** — driveable swerve drive on flat ground, inverse kinematics, teleoperation
3. **Suspension** — spring-damper prismatic joints modelling the splined-shaft suspension concept, ride quality tuning
4. **Terrain & obstacle traversal** — heightfield terrains (concrete, cracked sidewalk, curb cuts, grass, gravel), slope/tip-over analysis
5. **Control stack** — motor controller modelling, swerve drive controller, path following, odometry & state estimation
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
├── sim/                            # MuJoCo simulation (planned)
│   ├── models/                     #   MJCF model files
│   ├── controllers/                #   Swerve drive IK, path following
│   ├── terrains/                   #   Heightfield terrain definitions
│   └── tests/                      #   Simulation test scenarios
│
├── navigation/                     # Navigation & perception (planned)
│   ├── local_planner/              #   Obstacle avoidance
│   └── odometry/                   #   Wheel odometry + IMU fusion
│
└── config/                         # Shared configuration (planned)
```

## Related Projects

- **[Jugglebot](https://github.com/Project-DeepBlue-Juggling/Jugglebot)** — the robot that Ball Butler (v0) was originally built to serve. Shared configuration files (e.g. `hardware_config.h`) are generated by Jugglebot and consumed by this repo.
