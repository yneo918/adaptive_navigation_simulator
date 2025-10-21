"""Constants and configuration for teleop_core package."""

from enum import Enum


class RoverMode(Enum):
    """Rover operation modes."""
    NEUTRAL = "NEU_M"
    JOYSTICK = "JOY_M"
    NAVIGATION = "NAV_M"
    ADAPTIVE_NAVIGATION = "ADPTV_NAV_M"


# Velocity limits
MAX_VEL_TRANS = 1.0  # m/s
MAX_VEL_ROT = 0.5    # rad/s

# Default configuration
DEFAULT_N_ROVER = 6
DEFAULT_ROBOT_ID_PREFIX = "p"

# QoS settings
DEFAULT_QOS = 5
LOW_QOS = 1