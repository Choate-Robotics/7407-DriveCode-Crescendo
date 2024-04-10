from command.autonomous.custom_pathing import FollowPathCustom, AngleType
from command.autonomous.trajectory import CustomTrajectory, PoseType
from robot_systems import Robot, Field
from utils import POIPose
from command import *
import config
import math

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.LEFT_FOUR_NOTE_REVERSE.coords import (
    get_first_note,
    get_second_note,
    get_third_note,
    go_to_wing_boundary_1,
    go_to_wing_boundary_2,
    initial
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_first_note[0])),
        waypoints=[Translation2d(*coord) for coord in get_first_note[1]],
        end_pose=get_first_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.5,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=math.radians(-180)
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_third_note[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 0.5,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=math.radians(-180)
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_wing_boundary_1[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in go_to_wing_boundary_1[1]],
        end_pose=go_to_wing_boundary_1[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto,
        start_velocity=0,
        end_velocity=0,
        rev=False
    ),
    theta_f=AngleType.calculate
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_second_note[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=math.radians(-150)
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_wing_boundary_2[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in go_to_wing_boundary_2[1]],
        end_pose=go_to_wing_boundary_2[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto,
        start_velocity=0,
        end_velocity=0,
        rev=False
    ),
    theta_f=AngleType.calculate
)


auto = SequentialCommandGroup(
    ParallelCommandGroup(
        SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
        SequentialCommandGroup(
            ZeroWrist(Robot.wrist),
            ZeroElevator(Robot.elevator),

            InstantCommand(lambda: Field.odometry.enable()),
            # Shoot preload and deploy intake
            ParallelCommandGroup(
                ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
                DeployIntake(Robot.intake)
            ),
            InstantCommand(lambda: Field.odometry.disable()),

            # Get second note

            PathUntilIntake(path_1, Robot.wrist, Robot.intake),

            # Shoot second note
            InstantCommand(lambda: Field.odometry.enable()),
            ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
            InstantCommand(lambda: Field.odometry.disable()),

            # Get third note at midline
            PathUntilIntake(path_4, Robot.wrist, Robot.intake),

            # Drive back to wing
            path_5.raceWith(AimWrist(Robot.wrist, Field.calculations)),

            # Shoot third note
            InstantCommand(lambda: Field.odometry.enable()),
            ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
            InstantCommand(lambda: Field.odometry.disable()),

            # Get fourth note
            PathUntilIntake(path_2, Robot.wrist, Robot.intake),

            # Drive back to wing
            path_3.raceWith(AimWrist(Robot.wrist, Field.calculations)),

            # Shoot fourth note
            InstantCommand(lambda: Field.odometry.enable()),
            ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
            # InstantCommand(lambda: Field.odometry.disable()),

            SetWristIdle(Robot.wrist)

        )
    ),

    # path_1,
    # path_4,
    # path_5,
    # path_2,
    # path_3

)

routine = AutoRoutine(Pose2d(*initial), auto)