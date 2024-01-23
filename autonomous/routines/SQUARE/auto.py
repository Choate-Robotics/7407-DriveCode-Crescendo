from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot

from commands2 import (
    InstantCommand,
    SequentialCommandGroup
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.SQUARE.coords import (
    blue_team,
    drive_forward_1,
    drive_forward_2,
    initial,
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_forward_1[0]),
        waypoints=[Translation2d(*coord) for coord in drive_forward_1[1]],
        end_pose=Pose2d(*drive_forward_1[2]),
        max_velocity=0.2,
        max_accel=0.1,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_forward_2[0]),
        waypoints=[Translation2d(*coord) for coord in drive_forward_2[1]],
        end_pose=Pose2d(*drive_forward_2[2]),
        max_velocity=0.2,
        max_accel=0.1,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)

auto = SequentialCommandGroup(
    path_1,
    path_2,
    InstantCommand(lambda: print("Done")),
)

routine = AutoRoutine(Pose2d(*initial), auto, blue_team=blue_team)
