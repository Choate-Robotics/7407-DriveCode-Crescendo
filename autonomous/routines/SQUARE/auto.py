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
    drive_forward,
    drive_left,
    drive_back,
    drive_right,
    initial,
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_forward[0]),
        waypoints=[Translation2d(*coord) for coord in drive_forward[1]],
        end_pose=Pose2d(*drive_forward[2]),
        max_velocity=1.5,
        max_accel=0.4,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_left[0]),
        waypoints=[Translation2d(*coord) for coord in drive_left[1]],
        end_pose=Pose2d(*drive_left[2]),
        max_velocity=1.5,
        max_accel=0.4,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_back[0]),
        waypoints=[Translation2d(*coord) for coord in drive_back[1]],
        end_pose=Pose2d(*drive_back[2]),
        max_velocity=1.5,
        max_accel=0.4,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_right[0]),
        waypoints=[Translation2d(*coord) for coord in drive_right[1]],
        end_pose=Pose2d(*drive_right[2]),
        max_velocity=1.5,
        max_accel=0.4,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)

auto = SequentialCommandGroup(
    path_1,
    path_2,
    path_3,
    path_4,
    InstantCommand(lambda: print("Done")),
)

routine = AutoRoutine(Pose2d(*initial), auto, blue_team=blue_team)
