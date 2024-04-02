from command.autonomous.custom_pathing import FollowPathCustom, AngleType
from command.autonomous.trajectory import CustomTrajectory, PoseType
import command
from robot_systems import Robot, Field
from utils import POIPose
import config
import math
from command import *
import constants

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
    WaitCommand,
    ParallelRaceGroup,
    ParallelDeadlineGroup,
    WaitUntilCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.MIDLINE_NOTES.coords import *

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*shoot_first_note[0])),
        waypoints=[coord for coord in shoot_first_note[1]],
        end_pose=shoot_first_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.5,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=AngleType.calculate
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=get_second_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 0.5,
        start_velocity=0,
        end_velocity=0,
        rev=True,
        start_rotation=get_second_note[0].get().rotation().radians()
    ),
    theta_f=math.radians(-180)
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=shoot_second_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_second_note[1]],
        end_pose=shoot_second_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.5,
        start_velocity=0,
        end_velocity=0,
        rev=False,
        start_rotation=shoot_second_note[0].get().rotation().radians()
    ),
    theta_f=AngleType.calculate
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=get_third_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 0.5,
        start_velocity=0,
        end_velocity=2.5,
        rev=True,
        start_rotation=get_third_note[0].get().rotation().radians()
    ),
    theta_f=math.radians(120)
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=shoot_third_note[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_third_note[1]],
        end_pose=shoot_third_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.5,
        start_velocity=2.5,
        end_velocity=0,
        rev=False,
        start_rotation=shoot_third_note[0].get().rotation().radians()
    ),
    theta_f=AngleType.calculate
)

path_6 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=get_fourth_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in get_fourth_note[1]],
        end_pose=get_fourth_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 0.5,
        start_velocity=0,
        end_velocity=0,
        rev=True,
        start_rotation=get_fourth_note[0].get().rotation().radians()
    ),
    theta_f=math.radians(-180)
)

auto = ParallelCommandGroup(
    SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),

        # Drive to shot zone and deploy intake
        ParallelCommandGroup(
            path_1.raceWith(AimWrist(Robot.wrist, Field.calculations)),
            DeployIntake(Robot.intake),
        ),
        
        # Shoot first note
        InstantCommand(lambda: Field.odometry.enable_speaker_tags()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable_speaker_tags()),

        # get second note from midline
        PathUntilIntake(path_2, Robot.wrist, Robot.intake, 1),
        
        # drive to shot zone
        path_3.raceWith(AimWrist(Robot.wrist, Field.calculations)),
        
        # shoot second note
        InstantCommand(lambda: Field.odometry.enable_speaker_tags()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable_speaker_tags()),

        # get third note from midline
        # PathUntilIntake(path_4, Robot.wrist, Robot.intake),

        ParallelRaceGroup(
            SequentialCommandGroup(
                path_4,
                path_5
            ),
            SequentialCommandGroup(
                IntakeStageNote(Robot.wrist, Robot.intake),
                AimWrist(Robot.wrist, Field.calculations)
            )
        ),
        
        # drive to shot zone
        # path_5.raceWith(AimWrist(Robot.wrist, Field.calculations)),
        
        # shoot third note
        InstantCommand(lambda: Field.odometry.enable_speaker_tags()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable_speaker_tags()),

        # get fourth note from midline
        PathUntilIntake(path_6, Robot.wrist, Robot.intake)
    )
    # SequentialCommandGroup(
    #     path_1,
    #     path_2,
    #     path_3,
    #     path_4,
    #     path_5,
    #     path_6
    # )
)

routine = AutoRoutine(Pose2d(*initial), auto)
