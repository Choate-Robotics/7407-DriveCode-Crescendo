from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot

from commands2 import (
    InstantCommand,
    SequentialCommandGroup
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.TWO_NOTE.coords import (
    get_first_ring,
    get_second_ring,
    leave,
    initial,
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*get_first_ring[0]),
        waypoints=[Translation2d(*coord) for coord in get_first_ring[1]],
        end_pose=Pose2d(*get_first_ring[2]),
        max_velocity=3,
        max_accel=0.6,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*get_second_ring[0]),
        waypoints=[Translation2d(*coord) for coord in get_second_ring[1]],
        end_pose=Pose2d(*get_second_ring[2]),
        max_velocity=3,
        max_accel=0.6,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*leave[0]),
        waypoints=[Translation2d(*coord) for coord in leave[1]],
        end_pose=Pose2d(*leave[2]),
        max_velocity=3,
        max_accel=0.6,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

# Between paths, need to score rings
auto = SequentialCommandGroup(
    path_1,
    path_2,
    path_3,
    InstantCommand(lambda: print("Done")),
)

routine = AutoRoutine(Pose2d(*initial), auto)
