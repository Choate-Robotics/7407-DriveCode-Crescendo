from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
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
from autonomous.routines.LEFT_FOUR_NOTE.coords import (
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
        max_velocity=8,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_wing_boundary_1[0],
        waypoints=[coord for coord in go_to_wing_boundary_1[1]],
        end_pose=go_to_wing_boundary_1[2],
        max_velocity=8,
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
        max_velocity=8,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_wing_boundary_2[0],
        waypoints=[coord for coord in go_to_wing_boundary_2[1]],
        end_pose=go_to_wing_boundary_2[2],
        max_velocity=8,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False
    )
)


auto = ParallelCommandGroup(
    SetFlywheelLinearVelocity(Robot.flywheel, config.v0_flywheel),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),
        DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)),

        # Shoot preload and deploy intake
        ParallelCommandGroup(
            ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
            DeployIntake(Robot.intake)
        ),

        # Reset drivetrain
        ParallelCommandGroup(
            DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)),
            SetWristIdle(Robot.wrist),
        ),

        # Get second note
        ParallelCommandGroup(
            path_1,
            IntakeStageNote(Robot.wrist, Robot.intake).withTimeout(config.auto_intake_note_deadline)
        ),

        # Shoot second note
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

        # Reset drivetrain
        ParallelCommandGroup(
            DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)),
            SetWristIdle(Robot.wrist),
        ),

        # Get third note and go back to wing
        SequentialCommandGroup(
            ParallelCommandGroup(
                path_2,
                IntakeStageNote(Robot.wrist, Robot.intake).withTimeout(config.auto_intake_note_deadline),
            ),
            path_3
        ),

        # Shoot third note
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

        # Reset drivetrain
        ParallelCommandGroup(
            DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)),
            SetWristIdle(Robot.wrist),
        ),

        # Get fourth note
        SequentialCommandGroup(
            ParallelCommandGroup(
                path_4,
                IntakeStageNote(Robot.wrist, Robot.intake).withTimeout(config.auto_intake_note_deadline),
            ),
            path_5
        ),

        # Shoot fourth note
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

    )
)

routine = AutoRoutine(Pose2d(*initial), auto)