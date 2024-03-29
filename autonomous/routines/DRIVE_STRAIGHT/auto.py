from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot
import constants
import math

from commands2 import (
    InstantCommand,
    SequentialCommandGroup
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.DRIVE_STRAIGHT.coords import (
    drive_forward,
    drive_forward_2,
    initial,
)

from wpimath.geometry import Pose2d, Translation2d


vel = 4.5
accel = 4

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_forward[0]),
        waypoints=[Translation2d(*coord) for coord in drive_forward[1]],
        end_pose=Pose2d(*drive_forward[2]),
        max_velocity=vel,
        max_accel=accel,
        start_velocity=0,
        end_velocity=3,
        rev=True,
    ),
    theta_f=math.radians(180)
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_forward_2[0]),
        waypoints=[Translation2d(*coord) for coord in drive_forward_2[1]],
        end_pose=Pose2d(*drive_forward_2[2]),
        max_velocity=vel,
        max_accel=3,
        start_velocity=3,
        end_velocity=0,
        rev=True,
    ),
    theta_f=math.radians(180)
)

auto = SequentialCommandGroup(
    path_1,
    path_2
)

routine = AutoRoutine(Pose2d(*initial), auto)
