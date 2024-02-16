import command
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
from autonomous.routines.FIVE_NOTE.coords import *

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_first_ring[0])),
        waypoints=[Translation2d(*coord) for coord in get_first_ring[1]],
        end_pose=get_first_ring[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_second_ring[0],
        waypoints=[Translation2d(*coord) for coord in get_second_ring[1]],
        end_pose=get_second_ring[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_third_note[0],
        waypoints=[Translation2d(*coord) for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_midline_1[0],
        waypoints=[Translation2d(*coord) for coord in go_to_midline_1[1]],
        end_pose=go_to_midline_1[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_fourth_note[0],
        waypoints=[Translation2d(*coord) for coord in get_fourth_note[1]],
        end_pose=get_fourth_note[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_6 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_midline_2[0],
        waypoints=[Translation2d(*coord) for coord in go_to_midline_2[1]],
        end_pose=go_to_midline_2[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)

# Between paths, need to score rings
auto = SequentialCommandGroup(
    # command.ZeroElevator(Robot.elevator),
    WaitCommand(1),
    InstantCommand(lambda: print("Shot first note")),

    path_1,

    # command.RunIntake(Robot.intake),
    InstantCommand(lambda: print("Intaked second note")),
    WaitCommand(1),
    InstantCommand(lambda: print("Shot second note")),

    path_2,
    WaitCommand(0.75),
    InstantCommand(lambda: print("Intaked third note")),
    WaitCommand(1),
    InstantCommand(lambda: print("Shot third note")),

    path_3,
    WaitCommand(0.75),
    InstantCommand(lambda: print("Intaked fourth note")),


    path_4,
    InstantCommand(lambda: print("Went to midline")),
    WaitCommand(1),
    InstantCommand(lambda: print("Shot fourth note")),

    path_5,
    WaitCommand(0.75),
    InstantCommand(lambda: print("Intaked fifth note")),

    path_6,
    WaitCommand(1),
    InstantCommand(lambda: print("Shot fifth note")),
)

routine = AutoRoutine(Pose2d(*initial), auto)
