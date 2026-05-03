"""Buddy: high-level robotics package built on top of the STS3215 driver."""

from .arm import Arm, JointConfig, DEFAULT_JOINT_CONFIGS

__all__ = ["Arm", "JointConfig", "DEFAULT_JOINT_CONFIGS"]
