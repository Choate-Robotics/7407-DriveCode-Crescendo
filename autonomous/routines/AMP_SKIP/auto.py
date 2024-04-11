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
from autonomous.routines.AMP_SKIP.coords import (
    shoot_first_note,
    get_second_note,
    shoot_second_note,
    get_third_note,
    shoot_third_note,
    far_to_mid,
    mid_to_far,
    initial
)

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
    theta_f=math.radians(163)
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=get_second_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.5,
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
    theta_f=math.radians(163)
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=get_third_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.5,
        start_velocity=0,
        end_velocity=0,
        rev=True,
        start_rotation=get_third_note[0].get().rotation().radians()
    ),
    theta_f=math.radians(-135)
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=shoot_third_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_third_note[1]],
        end_pose=shoot_third_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.5,
        start_velocity=0,
        end_velocity=0,
        rev=False,
        start_rotation=shoot_third_note[0].get().rotation().radians()
    ),
    theta_f=math.radians(163)
)

path_far_to_mid = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=far_to_mid[0],
        waypoints=[Translation2d(*coord) for coord in far_to_mid[1]],
        end_pose=far_to_mid[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_mid_to_far = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=mid_to_far[0],
        waypoints=[coord for coord in mid_to_far[1]],
        end_pose=mid_to_far[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1.5,
        start_velocity=0,
        end_velocity=0,
        rev=False
    ),
)

auto = ParallelCommandGroup(
    SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),

        InstantCommand(lambda: Field.odometry.disable()),

        ParallelCommandGroup(
            path_1.raceWith(AimWrist(Robot.wrist, Field.calculations)),
            DeployIntake(Robot.intake),
        ),

        # Shoot first note
        InstantCommand(lambda: Field.odometry.enable()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable()),

        # Get second note
        PathUntilIntake(path_2, Robot.wrist, Robot.intake, 1),

        # Go to shot location
        path_3.raceWith(AimWrist(Robot.wrist, Field.calculations)),

        # Shoot second note
        InstantCommand(lambda: Field.odometry.enable()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable()),

        # Get third note
        PathUntilIntake(path_4, Robot.wrist, Robot.intake),

        # Go to shot location
        path_5.raceWith(AimWrist(Robot.wrist, Field.calculations)),

        # Shoot third note
        InstantCommand(lambda: Field.odometry.enable()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable()),

    ),

    # SequentialCommandGroup(
    #     InstantCommand(lambda: Field.odometry.disable()),
    #     path_1,
    #     path_2,
    #     path_3,
    #     path_4,
    #     path_5,
    # )
    # path_far_to_mid,
    # path_mid_to_far

)

routine = AutoRoutine(Pose2d(*initial), auto)