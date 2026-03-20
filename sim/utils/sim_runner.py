"""Headless simulation stepping with swerve IK control.

Used by validation tests to drive the rover through scripted scenarios
and record time-series data.
"""

import mujoco
import numpy as np

from controllers.swerve_ik import swerve_ik
from utils.constants import PREFIXES
from utils.transforms import quat_to_euler
from utils.model_helpers import get_wheel_body_ids


def step_with_control(model, data, ids, vx, vy, omega, duration_s,
                      control_decimation=10, record_wheels=False):
    """Step the simulation with swerve IK control for a given duration.

    Args:
        model:               MjModel instance.
        data:                MjData instance.
        ids:                 Tuple from get_ids().
        vx, vy, omega:       Body-frame velocity command.
        duration_s:          How long to simulate (seconds).
        control_decimation:  Physics steps per control step (default 10).
        record_wheels:       If True, also record per-wheel world positions
                             (keys: ``{prefix}_wx/wy/wz``).

    Returns:
        Dict of NumPy arrays keyed by signal name.
    """
    yaw_jnt, drv_jnt, susp_jnt, yaw_act, drv_act, chassis_free = ids
    yaw_qpos_adr = [model.jnt_qposadr[j] for j in yaw_jnt]
    susp_qpos_adr = [model.jnt_qposadr[j] for j in susp_jnt]
    chassis_adr = model.jnt_qposadr[chassis_free]

    wheel_body_ids = get_wheel_body_ids(model) if record_wheels else None

    n_steps = int(duration_s / model.opt.timestep)

    records = {
        "time": [], "cx": [], "cy": [], "cz": [],
        "roll": [], "pitch": [], "heading": [],
    }
    for p in PREFIXES:
        records[f"{p}_susp"] = []
        if record_wheels:
            records[f"{p}_wx"] = []
            records[f"{p}_wy"] = []
            records[f"{p}_wz"] = []

    for step in range(n_steps):
        if step % control_decimation == 0:
            current_yaw = [data.qpos[a] for a in yaw_qpos_adr]
            modules = swerve_ik(vx, vy, omega, current_yaw)
            for i in range(4):
                data.ctrl[yaw_act[i]] = modules[i].yaw_angle
                data.ctrl[drv_act[i]] = modules[i].wheel_speed

            # Record chassis pose
            q = data.qpos
            cx, cy, cz = q[chassis_adr], q[chassis_adr + 1], q[chassis_adr + 2]
            qw, qx, qy, qz = (q[chassis_adr + 3], q[chassis_adr + 4],
                                q[chassis_adr + 5], q[chassis_adr + 6])
            roll, pitch, heading = quat_to_euler(qw, qx, qy, qz)

            records["time"].append(data.time)
            records["cx"].append(cx)
            records["cy"].append(cy)
            records["cz"].append(cz)
            records["roll"].append(roll)
            records["pitch"].append(pitch)
            records["heading"].append(heading)

            for i, p in enumerate(PREFIXES):
                records[f"{p}_susp"].append(q[susp_qpos_adr[i]])
                if record_wheels:
                    wp = data.xpos[wheel_body_ids[i]]
                    records[f"{p}_wx"].append(wp[0])
                    records[f"{p}_wy"].append(wp[1])
                    records[f"{p}_wz"].append(wp[2])

        mujoco.mj_step(model, data)

    return {k: np.array(v) for k, v in records.items()}
