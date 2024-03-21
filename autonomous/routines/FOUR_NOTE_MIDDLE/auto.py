from command.autonomous.custom_pathing import FollowPathCustom
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
        # start_pose=PoseType.current,
        waypoints=[Translation2d(*coord) for coord in get_first_note[1]],
        end_pose=get_first_note[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=math.radians(-135)
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_second_note[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=False
    ),
    theta_f=math.radians(135)
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_third_note[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=False
    ),
    theta_f=math.radians(135)
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_midline[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in go_to_midline[1]],
        end_pose=go_to_midline[2],
        max_velocity=10,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=math.radians(-180)
)

auto = ParallelCommandGroup(
    SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),

        # Shoot first note preload and deploy intake
        ParallelCommandGroup(
            ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
            DeployIntake(Robot.intake)
        ),
        SetWristIdle(Robot.wrist).withTimeout(2),
        # Get second note
        ParallelCommandGroup(
            path_1,
            IntakeStageNote(Robot.wrist, Robot.intake).withTimeout(config.auto_intake_note_deadline)
        ),

        # Shoot second note
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        ParallelCommandGroup(
            # DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)).withTimeout(3),
            SetWristIdle(Robot.wrist).withTimeout(2),
        ),

        # Get third note
        ParallelCommandGroup(
            path_2,
            IntakeStageNote(Robot.wrist, Robot.intake).withTimeout(config.auto_intake_note_deadline)
        ),

        # Shoot third note
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        SetWristIdle(Robot.wrist).withTimeout(2),
        # Get fourth note
        ParallelCommandGroup(
            path_3,
            IntakeStageNote(Robot.wrist, Robot.intake).withTimeout(config.auto_intake_note_deadline)
            
        ),

        # Shoot fourth note
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

        ParallelCommandGroup(
            # DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)).withTimeout(3),
            SetWristIdle(Robot.wrist).withTimeout(2),
        ),

        ParallelCommandGroup(
            path_4,
            IntakeStageNote(Robot.wrist, Robot.intake).withTimeout(config.auto_intake_note_deadline)
        )
    )
)

routine = AutoRoutine(Pose2d(*initial), auto)
