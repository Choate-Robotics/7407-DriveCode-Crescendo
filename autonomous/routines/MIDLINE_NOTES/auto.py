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
        max_velocity=config.drivetrain_max_vel_auto - 1,
        max_accel=config.drivetrain_max_accel_auto - 1.25,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=AngleType.calculate
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_second_note[0])),
        # start_pose=PoseType.current,
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.25,
        start_velocity=0,
        end_velocity=1,
        rev=True,
        start_rotation=get_second_note[0][2]
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
        max_accel=config.drivetrain_max_accel_auto - 1.25,
        start_velocity=0,
        end_velocity=1.5,
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
        max_accel=config.drivetrain_max_accel_auto - 1.25,
        start_velocity=0,
        end_velocity=1,
        rev=True,
        start_rotation=get_third_note[0].get().rotation().radians()
    ),
    theta_f=math.radians(130)
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=shoot_third_note[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_third_note[1]],
        end_pose=shoot_third_note[2],
        max_velocity=config.drivetrain_max_vel_auto - 1.5,
        max_accel=config.drivetrain_max_accel_auto - 1.25,
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
        start_pose=get_fourth_note[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in get_fourth_note[1]],
        end_pose=get_fourth_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.25,
        start_velocity=0,
        end_velocity=0,
        rev=True,
        start_rotation=get_fourth_note[0].get().rotation().radians()
    ),
    theta_f=math.radians(-180)
)

path_7 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=shoot_fourth_note[0],
        # start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_fourth_note[1]],
        end_pose=shoot_fourth_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1,
        start_velocity=0,
        end_velocity=0,
        rev=False,
        start_rotation=shoot_fourth_note[0].get().rotation().radians()
    ),
    theta_f=AngleType.calculate
)

path_8 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=come_back_with_third[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in come_back_with_third[1]],
        end_pose=come_back_with_third[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.25,
        start_velocity=0,
        end_velocity=1.5,
        rev=False,
        start_rotation=come_back_with_third[0].get().rotation().radians()
    ),
    theta_f=AngleType.calculate
)

auto = ParallelCommandGroup(
    SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),

        # Drive to shot zone and deploy intake
        InstantCommand(lambda: Field.odometry.disable()),
        DeployIntake(Robot.intake).withTimeout(0.3),
        PassNote(Robot.wrist),

        # get second note from midline
        ParallelRaceGroup(
            SequentialCommandGroup(
                path_2,
                path_3
            ),
            SequentialCommandGroup(
                IntakeStageNote(Robot.wrist, Robot.intake),
                AimWrist(Robot.wrist, Field.calculations)
            )
        ),

        # shoot second note
        InstantCommand(lambda: Field.odometry.enable()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable()),

        # get third note from midline
        # PathUntilIntake(path_4, Robot.wrist, Robot.intake),

        ParallelRaceGroup(
            SequentialCommandGroup(
                path_4,
                path_8
            ),
            SequentialCommandGroup(
                IntakeStageNote(Robot.wrist, Robot.intake),
                AimWrist(Robot.wrist, Field.calculations)
            )
        ),

        # drive to shot zone
        # path_8.raceWith(AimWrist(Robot.wrist, Field.calculations)),

        # shoot third note
        InstantCommand(lambda: Field.odometry.enable()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        # InstantCommand(lambda: Field.odometry.disable()),

        # # get fourth note from midline
        # PathUntilIntake(path_6, Robot.wrist, Robot.intake),
        
        # path_7.raceWith(AimWrist(Robot.wrist, Field.calculations)),
        
        # InstantCommand(lambda: Field.odometry.enable()),
        # ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations)
    )
    # SequentialCommandGroup(
    #     # path_1,
    #     InstantCommand(lambda: Field.odometry.disable()),
    #     path_2,
    #     path_3,
    #     path_4,
    #     path_8,
    #     # path_5,
    #     # path_6,
    #     # path_7
    # )
)

routine = AutoRoutine(Pose2d(initial[0], initial[1], math.radians(-120)), auto)
