import math

from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from wpimath.geometry import Pose2d, Translation2d

import command
import config
import constants

from coords import start_rotating
from command.autonomous.custom_pathing import RotateInPlace
from command.autonomous.trajectory import CustomTrajectory

from robot_systems import Robot, Sensors
from units.SI import meters_per_second, meters_per_second_squared

max_vel: meters_per_second = 3
max_accel: meters_per_second_squared = 2

path_1 = 