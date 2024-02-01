from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot
from utils import POIPose

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.FOUR_NOTE.coords import (
    get_first_note,
    get_second_note,
    get_third_note,
    initial
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_first_note[0])),
        waypoints=[Translation2d(*coord) for coord in get_first_note[1]],
        end_pose=get_first_note[2],
        max_velocity=1,
        max_accel=0.5,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_second_note[0],
        waypoints=[Translation2d(*coord) for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=5,
        max_accel=0.5,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_third_note[0],
        waypoints=[Translation2d(*coord) for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=1,
        max_accel=0.5,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

auto = SequentialCommandGroup(
    path_1,
    WaitCommand(1),

    path_2,
    WaitCommand(1),

    path_3,
    WaitCommand(1)
)

routine = AutoRoutine(Pose2d(*initial), auto)