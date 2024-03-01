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
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.FOUR_NOTE_MIDDLE.coords import (
    get_first_note,
    get_second_note,
    get_third_note,
    go_to_midline,
    initial
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_first_note[0])),
        waypoints=[Translation2d(*coord) for coord in get_first_note[1]],
        end_pose=get_first_note[2],
        max_velocity=5,
        max_accel=2,
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
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=False
    )
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_third_note[0],
        waypoints=[coord for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=False
    )
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_midline[0],
        waypoints=[coord for coord in go_to_midline[1]],
        end_pose=go_to_midline[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

auto = ParallelCommandGroup(
    # SetFlywheelLinearVelocity(Robot.flywheel, config.v0_flywheel),
    SequentialCommandGroup(
        # ZeroWrist(Robot.wrist),
        # ZeroElevator(Robot.elevator),

        # # Shoot first note preload and deploy intake
        # ParallelCommandGroup(
        #     ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        #     DeployIntake(Robot.intake)
        # ),
        
        # # Get second note
        # ParallelCommandGroup(
            path_1,
        #     SequentialCommandGroup(
        #         SetWristIdle(Robot.wrist),
        #         IntakeStageNote(Robot.wrist, Robot.intake)
        #     )
        # ),

        # # Shoot second note
        # ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        # ParallelCommandGroup(
        #     DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)),
        #     SetWristIdle(Robot.wrist),
        # ),

        # # Get third note
        # ParallelCommandGroup(
            path_2,
        #     IntakeStageNote(Robot.wrist, Robot.intake)
        # ),

        # # Shoot third note
        # ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

        # # Get fourth note
        # ParallelCommandGroup(
            path_3,
        #     SequentialCommandGroup(
        #         SetWristIdle(Robot.wrist),
        #         IntakeStageNote(Robot.wrist, Robot.intake)
        #     )
            
        # ),

        # # Shoot fourth note
        # ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

        # ParallelCommandGroup(
        #     DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)),
        #     SetWristIdle(Robot.wrist),
        # ),

        # ParallelCommandGroup(
            path_4,
        #     IntakeStageNote(Robot.wrist, Robot.intake)
        # )
    )
)

routine = AutoRoutine(Pose2d(*initial), auto)