from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
import command
from robot_systems import Robot, Field
from utils import POIPose
import config
import math

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.MIDLINE_NOTES.coords import *

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=shoot_first_note[0],
        waypoints=[coord for coord in shoot_first_note[1]],
        end_pose=shoot_first_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_second_note[0],
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=shoot_second_note[0],
        waypoints=[coord for coord in shoot_second_note[1]],
        end_pose=shoot_second_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False
    )
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_third_note[0],
        waypoints=[coord for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=shoot_third_note[0],
        waypoints=[coord for coord in shoot_third_note[1]],
        end_pose=shoot_third_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False
    )
)

path_6 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_fourth_note[0],
        waypoints=[coord for coord in get_fourth_note[1]],
        end_pose=get_fourth_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

auto = SequentialCommandGroup(
    path_1,
    path_2,
    path_3,
    path_4,
    path_5,
    path_6
)

routine = AutoRoutine(initial.get(), auto)
