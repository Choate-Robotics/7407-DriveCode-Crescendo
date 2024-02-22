import config
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from command import (
    ZeroWrist,
    ZeroElevator,
    ShootAuto,
    DeployIntake,
    RunIntake
)

from robot_systems import Robot, Field
from utils import POIPose

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    WaitCommand,
    ParallelCommandGroup,
    ParallelRaceGroup,
    PrintCommand,
    WaitUntilCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.FIVE_NOTE.coords import *

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_second_note[0])),
        waypoints=[Translation2d(*coord) for coord in get_second_note[1]],
        end_pose=get_second_note[2],
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

path_3 = FollowPathCustom(
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
        start_pose=get_fifth_note[0],
        waypoints=[Translation2d(*coord) for coord in get_fifth_note[1]],
        end_pose=get_fifth_note[2],
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

auto = SequentialCommandGroup(
    ZeroWrist(Robot.wrist),
    ZeroElevator(Robot.elevator),

    # Shoot first note
    ParallelCommandGroup(
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        DeployIntake(Robot.intake)
    ),

    # Get second note
    ParallelCommandGroup(
        path_1,
        RunIntake(Robot.intake)
    ),

    # Shoot second note
    ShootAuto(
        Robot.drivetrain,
        Robot.wrist,
        Robot.flywheel,
        Field.calculations
    ),

    # Get third note
    ParallelCommandGroup(
        path_2,
        RunIntake(Robot.intake)
    ),

    # Shoot third note
    ShootAuto(
        Robot.drivetrain,
        Robot.wrist,
        Robot.flywheel,
        Field.calculations
    ),

    # Get fourth note
    ParallelCommandGroup(
        path_3,
        RunIntake(Robot.intake)
    ),

    # Go to midline to shoot fourth note
    path_4,

    # Shoot fourth note
    ShootAuto(
        Robot.drivetrain,
        Robot.wrist,
        Robot.flywheel,
        Field.calculations
    ),

    # Get fifth note
    ParallelCommandGroup(
        path_5,
        RunIntake(Robot.intake)
    ),

    path_6,

    ShootAuto(
        Robot.drivetrain,
        Robot.wrist,
        Robot.flywheel,
        Field.calculations
    )
)

routine = AutoRoutine(Pose2d(*initial), auto)
