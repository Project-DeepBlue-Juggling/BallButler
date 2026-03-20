"""BallButler Rover — MuJoCo Simulation Entry Point.

Launches an interactive MuJoCo viewer with:
  - A 4-wheel swerve drive rover with spring-damper suspension
  - 500 Hz physics (implicitfast integrator), 50 Hz control, 60 FPS rendering
  - Swerve drive inverse kinematics (global-frame velocity -> per-wheel commands)
  - Keyboard / gamepad teleoperation
  - CSV data logging (commands, yaw angles, wheel speeds, suspension, chassis pose)

Velocity commands from the input handler are interpreted in the **global frame**
(world x/y axes).  Before feeding into the swerve IK (which expects body-frame
velocities), they are rotated by the negative of the current chassis heading.

Scenes:
  Default (scene.xml)  — flat ground + Phase 2 bumps / curb-cut ramp.
  Terrain (--terrain)  — multi-terrain course (concrete, curb cuts, cracked
                         sidewalk, grass, gravel) + slope analysis area.

Usage:
    cd sim && python main.py             # default scene
    cd sim && python main.py --terrain   # Phase 3 terrain course
"""

import math
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

from config import SimConfig, SCENE_XML, TERRAIN_SCENE_XML
from input_handler import InputHandler, VelocityCommand
from data_logger import DataLogger
from controllers.swerve_ik import swerve_ik
from utils.constants import (
    PREFIXES, YAW_JOINT_NAMES, DRIVE_JOINT_NAMES,
    SUSP_JOINT_NAMES, YAW_ACT_NAMES, DRIVE_ACT_NAMES,
)
from utils.transforms import quat_to_euler


def _get_qpos_adr(model):
    """Pre-compute qpos/qvel addresses for fast access in the control loop."""
    def jid(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

    def aid(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    yaw_jnt_ids = [jid(n) for n in YAW_JOINT_NAMES]
    drv_jnt_ids = [jid(n) for n in DRIVE_JOINT_NAMES]
    susp_jnt_ids = [jid(n) for n in SUSP_JOINT_NAMES]
    yaw_act_ids = [aid(n) for n in YAW_ACT_NAMES]
    drv_act_ids = [aid(n) for n in DRIVE_ACT_NAMES]
    chassis_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

    yaw_qpos_adr = [model.jnt_qposadr[j] for j in yaw_jnt_ids]
    drv_qvel_adr = [model.jnt_dofadr[j] for j in drv_jnt_ids]
    susp_qpos_adr = [model.jnt_qposadr[j] for j in susp_jnt_ids]
    chassis_qpos_adr = model.jnt_qposadr[jid("chassis_free")]

    return {
        "yaw_act_ids": yaw_act_ids,
        "drv_act_ids": drv_act_ids,
        "yaw_qpos_adr": yaw_qpos_adr,
        "drv_qvel_adr": drv_qvel_adr,
        "susp_qpos_adr": susp_qpos_adr,
        "chassis_qpos_adr": chassis_qpos_adr,
        "chassis_body": chassis_body,
    }


def _global_to_body(vx_global, vy_global, heading):
    """Rotate a global-frame velocity command into the body frame.

    Args:
        vx_global: Velocity along world x-axis (m/s).
        vy_global: Velocity along world y-axis (m/s).
        heading:   Current chassis heading (rad), CCW positive from world x.

    Returns:
        (vx_body, vy_body) in the chassis frame.
    """
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    vx_body = vx_global * cos_h + vy_global * sin_h
    vy_body = -vx_global * sin_h + vy_global * cos_h
    return vx_body, vy_body


def run():
    cfg = SimConfig()

    # --- Select scene ---
    use_terrain = "--terrain" in sys.argv
    if use_terrain:
        from terrains.terrain_generator import populate_terrain_heightfields
        scene_path = TERRAIN_SCENE_XML
    else:
        scene_path = SCENE_XML

    # --- Load MuJoCo model and data ---
    model = mujoco.MjModel.from_xml_path(str(scene_path))

    if use_terrain:
        populate_terrain_heightfields(model)

    data = mujoco.MjData(model)

    assert abs(model.opt.timestep - cfg.timestep) < 1e-9, (
        f"MJCF timestep ({model.opt.timestep}) != config ({cfg.timestep})"
    )

    adr = _get_qpos_adr(model)

    input_handler = InputHandler(cfg)

    # Logging fields
    extra_fields = []
    for prefix in PREFIXES:
        extra_fields.extend([
            f"{prefix}_yaw_cmd", f"{prefix}_yaw_pos",
            f"{prefix}_drive_cmd", f"{prefix}_drive_vel",
            f"{prefix}_susp_pos",
        ])
    extra_fields.extend([
        "chassis_x", "chassis_y", "chassis_z",
        "chassis_roll", "chassis_pitch", "chassis_heading",
    ])

    logger = DataLogger(cfg, extra_fields=extra_fields)

    scene_label = "terrain course" if use_terrain else "Phase 2 (bumps + ramp)"
    print(f"[main] Scene           : {scene_label}")
    print(f"[main] Physics timestep : {cfg.timestep * 1000:.1f} ms ({1/cfg.timestep:.0f} Hz)")
    print(f"[main] Integrator      : implicitfast")
    print(f"[main] Control rate     : {cfg.control_rate_hz:.0f} Hz (decimation {cfg.control_decimation})")
    print(f"[main] Render target    : {cfg.render_fps:.0f} FPS")
    print(f"[main] Suspension       : k=3600 N/m, c=100 N-s/m, travel=+-25 mm")
    print(f"[main] Velocity frame   : global (world x/y axes)")
    print("[main] Controls (MuJoCo viewer window):")
    print("       Up/Down    = forward/back     (0.25 m/s per tap, world +x/-x)")
    print("       Left/Right = strafe left/right (0.25 m/s per tap, world +y/-y)")
    print("       , / .      = rotate CCW/CW    (0.5 rad/s per tap)")
    print("       Space      = stop all")
    if use_terrain:
        print("[main] Terrain course: concrete -> curb cut -> sidewalk -> grass -> gravel")
        print("[main] Slopes at y=5: 5deg, 10deg, 15deg, 20deg, 25deg")
    else:
        print("[main] Test terrain: bumps at x=3..5 m, curb-cut ramp at x=6.5..8.5 m")
    print("[main] Starting simulation...")

    cmd = VelocityCommand()

    # --- Launch the interactive viewer ---
    try:
        with mujoco.viewer.launch_passive(
            model, data,
            key_callback=input_handler.key_callback,
        ) as viewer:
            # Camera tracks the chassis
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = adr["chassis_body"]
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25

            step_count = 0

            while viewer.is_running():
                step_start = time.monotonic()

                # --- Control at 50 Hz ---
                if step_count % cfg.control_decimation == 0:
                    cmd = input_handler.poll()

                    # Read chassis heading for global -> body frame transform
                    ca = adr["chassis_qpos_adr"]
                    qw = data.qpos[ca + 3]
                    qx = data.qpos[ca + 4]
                    qy = data.qpos[ca + 5]
                    qz = data.qpos[ca + 6]
                    roll, pitch, heading = quat_to_euler(qw, qx, qy, qz)

                    # Transform global-frame velocity command to body frame
                    vx_body, vy_body = _global_to_body(cmd.vx, cmd.vy, heading)

                    # Read current yaw angles
                    current_yaw = [data.qpos[a] for a in adr["yaw_qpos_adr"]]

                    # Swerve IK: body-frame velocity -> module commands
                    modules = swerve_ik(vx_body, vy_body, cmd.omega, current_yaw)

                    # Apply actuator commands
                    for i in range(4):
                        data.ctrl[adr["yaw_act_ids"][i]] = modules[i].yaw_angle
                        data.ctrl[adr["drv_act_ids"][i]] = modules[i].wheel_speed

                    # --- Logging ---
                    cx = data.qpos[ca + 0]
                    cy = data.qpos[ca + 1]
                    cz = data.qpos[ca + 2]

                    record = {
                        "sim_time": data.time,
                        "wall_time": time.monotonic(),
                        "cmd_vx": cmd.vx,
                        "cmd_vy": cmd.vy,
                        "cmd_omega": cmd.omega,
                        "chassis_x": cx,
                        "chassis_y": cy,
                        "chassis_z": cz,
                        "chassis_roll": roll,
                        "chassis_pitch": pitch,
                        "chassis_heading": heading,
                    }
                    for i, p in enumerate(PREFIXES):
                        record[f"{p}_yaw_cmd"] = modules[i].yaw_angle
                        record[f"{p}_yaw_pos"] = current_yaw[i]
                        record[f"{p}_drive_cmd"] = modules[i].wheel_speed
                        record[f"{p}_drive_vel"] = data.qvel[adr["drv_qvel_adr"][i]]
                        record[f"{p}_susp_pos"] = data.qpos[adr["susp_qpos_adr"][i]]

                    logger.log(record)

                # Step physics
                mujoco.mj_step(model, data)
                step_count += 1

                # Sync viewer at render rate
                if step_count % cfg.render_decimation == 0:
                    viewer.sync()

                    # Throttle to roughly real-time
                    elapsed = time.monotonic() - step_start
                    target_dt = cfg.render_decimation * cfg.timestep
                    if elapsed < target_dt:
                        time.sleep(target_dt - elapsed)

    except KeyboardInterrupt:
        print("\n[main] Interrupted by user.")
    finally:
        logger.close()
        input_handler.close()
        print("[main] Shutdown complete.")


if __name__ == "__main__":
    run()
