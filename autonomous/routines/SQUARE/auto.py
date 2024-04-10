import math
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory, PoseType
from robot_systems import Robot, Field

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.SQUARE.coords import (
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
        max_velocity=5,
        max_accel=1,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    theta_f=math.radians(45)
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_left[0]),
        # start_pose=PoseType.current,
        waypoints=[Translation2d(*coord) for coord in drive_left[1]],
        end_pose=Pose2d(*drive_left[2]),
        max_velocity=5,
        max_accel=1,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    theta_f=math.radians(90)
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_back[0]),
        # start_pose=PoseType.current,
        waypoints=[Translation2d(*coord) for coord in drive_back[1]],
        end_pose=Pose2d(*drive_back[2]),
        max_velocity=5,
        max_accel=1,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    theta_f=math.radians(135)
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*drive_right[0]),
        # start_pose=PoseType.current,
        waypoints=[Translation2d(*coord) for coord in drive_right[1]],
        end_pose=Pose2d(*drive_right[2]),
        max_velocity=5,
        max_accel=1,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    theta_f=math.radians(180)
)

auto = SequentialCommandGroup(
    InstantCommand(lambda: Field.odometry.disable()),
    path_1,
    path_2,
    path_3,
    path_4,
    InstantCommand(lambda: print("Done")),
)

routine = AutoRoutine(Pose2d(*initial), auto)
