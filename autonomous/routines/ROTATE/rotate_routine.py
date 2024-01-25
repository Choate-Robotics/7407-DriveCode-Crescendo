from autonomous.auto_routine import AutoRoutine
from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
)
from wpimath.geometry import Pose2d

from autonomous.routines.DRIVE_STRAIGHT.coords import (
    blue_team,
    initial,
)
from robot_systems import Robot
from units.SI import meters_per_second, meters_per_second_squared

import math

max_vel: meters_per_second = 3
max_accel: meters_per_second_squared = 2


# path_1 = RotateInPlace(
#     subsystem=Robot.drivetrain,
#     theta_f=math.pi/2
# )
#
# auto = SequentialCommandGroup(
#     path_1,
#     InstantCommand(lambda: print("Done")),
# )

# routine = AutoRoutine(Pose2d(*initial), auto, blue_team=blue_team)