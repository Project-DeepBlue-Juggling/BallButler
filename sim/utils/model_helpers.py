"""MuJoCo model loading and ID lookup helpers."""

import mujoco

from config import SCENE_XML, TERRAIN_SCENE_XML
from utils.constants import (
    YAW_JOINT_NAMES,
    DRIVE_JOINT_NAMES,
    SUSP_JOINT_NAMES,
    YAW_ACT_NAMES,
    DRIVE_ACT_NAMES,
    WHEEL_BODY_NAMES,
)


def get_ids(model):
    """Look up joint, actuator, and body indices from the model.

    Returns:
        (yaw_jnt, drv_jnt, susp_jnt, yaw_act, drv_act, chassis_free)
        where chassis_free is the joint ID of the chassis freejoint.
    """
    def jid(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

    def aid(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    yaw_jnt = [jid(n) for n in YAW_JOINT_NAMES]
    drv_jnt = [jid(n) for n in DRIVE_JOINT_NAMES]
    susp_jnt = [jid(n) for n in SUSP_JOINT_NAMES]
    yaw_act = [aid(n) for n in YAW_ACT_NAMES]
    drv_act = [aid(n) for n in DRIVE_ACT_NAMES]
    chassis_free = jid("chassis_free")
    return yaw_jnt, drv_jnt, susp_jnt, yaw_act, drv_act, chassis_free


def get_wheel_body_ids(model):
    """Look up wheel body indices from the model."""
    return [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
        for n in WHEEL_BODY_NAMES
    ]


def load_model():
    """Load the default scene (flat ground + Phase 2 test terrain)."""
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    return model, data


def load_terrain_model():
    """Load the terrain course scene and populate heightfield data."""
    from terrains.terrain_generator import populate_terrain_heightfields

    model = mujoco.MjModel.from_xml_path(str(TERRAIN_SCENE_XML))
    populate_terrain_heightfields(model)
    data = mujoco.MjData(model)
    return model, data
