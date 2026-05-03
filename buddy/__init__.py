"""Buddy: high-level robotics package built on top of the STS3215 driver."""

from .arm import Arm, JointConfig, DEFAULT_JOINT_CONFIGS
from .kinematics import (
    DEFAULT_DH_PARAMS,
    DEFAULT_LINKS,
    KinematicsError,
    NUM_JOINTS,
    dh_table,
    forward,
    inverse,
)

__all__ = [
    "Arm",
    "JointConfig",
    "DEFAULT_JOINT_CONFIGS",
    "DEFAULT_DH_PARAMS",
    "DEFAULT_LINKS",
    "KinematicsError",
    "NUM_JOINTS",
    "dh_table",
    "forward",
    "inverse",
]
