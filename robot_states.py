import config, constants
from enum import Enum
drivetrain_controlled_vel = constants.drivetrain_max_vel
drivetrain_controlled_angular_vel = constants.drivetrain_max_angular_vel

# CLIMBING
ready_to_climb: bool = False
climbing: bool = False
climbed: bool = False

# AMP

class FlywheelState(Enum):
    
    idle = 0
    shooting = 1
    amping = 2
    manual = 3
    released = 4
    feeding = 5

flywheel_state = FlywheelState.idle

flywheel_tolerance = config.flywheel_min_shot_tolerance