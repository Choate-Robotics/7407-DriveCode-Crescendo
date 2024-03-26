# import math

from commands2 import (  # ParallelCommandGroup,; WaitCommand,
    InstantCommand,
    SequentialCommandGroup,
)
from wpimath.geometry import Pose2d

# import config
from autonomous.auto_routine import AutoRoutine
from autonomous.routines.ROTATE.coords import initial, start_rotating

# from command import *
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot

# max_vel: meters_per_second = 3
# max_accel: meters_per_second_squared = 2


# path_1 = RotateInPlace(
#     subsystem=Robot.drivetrain,
#     theta_f=math.pi/2
# )
#
auto = SequentialCommandGroup(
    start_rotating,
    InstantCommand(lambda: print("Done")),
)
path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=POIPose(Pose2d(*get_first_note[0])),
        start_pose=initial,
        waypoints=[],
        end_pose=start_rotating[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
)

auto = SequentialCommandGroup(path_1, InstantCommand(lambda: print("Done")))
routine = AutoRoutine(Pose2d(*initial), auto)
