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
    WaitCommand,
    ParallelRaceGroup
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.AMP_SKIP_2_2.coords import (
    shoot_first_note,
    get_second_note,
    shoot_second_note,
    get_third_note,
    shoot_third_note,
    get_fourth_note,
    far_to_mid,
    mid_to_far,
    initial
)

from wpimath.geometry import Pose2d, Translation2d

# path_1 = FollowPathCustom(
#     subsystem=Robot.drivetrain,
#     trajectory=CustomTrajectory(
#         start_pose=shoot_first_note[0],
#         waypoints=[coord for coord in shoot_first_note[1]],
#         end_pose=shoot_first_note[2],
#         max_velocity=config.drivetrain_max_vel_auto,
#         max_accel=config.drivetrain_max_accel_auto - 1,
#         start_velocity=0,
#         end_velocity=0,
#         rev=True
#     ),
#     theta_f=math.radians(-120)
# )

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_second_note[0])),
        # start_pose=PoseType.current,
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1,
        start_velocity=0,
        end_velocity=0,
        rev=True,
        start_rotation=get_second_note[0][2]
    ),
    theta_f=math.radians(-155)
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=shoot_second_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_second_note[1]],
        end_pose=shoot_second_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1,
        start_velocity=0,
        end_velocity=0,
        rev=False,
        start_rotation=shoot_third_note[0].get().rotation().radians()
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
        max_accel=config.drivetrain_max_accel_auto - 1,
        start_velocity=0,
        end_velocity=1,
        rev=True,
        start_rotation=math.radians(-180)
    ),
    theta_f=math.radians(-180)
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=shoot_third_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_third_note[1]],
        end_pose=shoot_third_note[2],
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 1,
        start_velocity=0,
        end_velocity=0,
        rev=False,
        start_rotation=math.radians(-180)
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
        max_accel=config.drivetrain_max_accel_auto - 1,
        start_velocity=0,
        end_velocity=0,
        rev=True,
        start_rotation=get_fourth_note[0].get().rotation().radians()
    ),
    theta_f=(math.radians(-180))
)

# path_far_to_mid = FollowPathCustom(
#     subsystem=Robot.drivetrain,
#     trajectory=CustomTrajectory(
#         start_pose=far_to_mid[0],
#         waypoints=[Translation2d(*coord) for coord in far_to_mid[1]],
#         end_pose=far_to_mid[2],
#         max_velocity=12,
#         max_accel=3,
#         start_velocity=0,
#         end_velocity=0,
#         rev=False,
#     ),
#     period=0.03,
# )
#
# path_mid_to_far = FollowPathCustom(
#     subsystem=Robot.drivetrain,
#     trajectory=CustomTrajectory(
#         start_pose=mid_to_far[0],
#         waypoints=[coord for coord in mid_to_far[1]],
#         end_pose=mid_to_far[2],
#         max_velocity=config.drivetrain_max_vel_auto,
#         max_accel=config.drivetrain_max_accel_auto - 1.5,
#         start_velocity=0,
#         end_velocity=0,
#         rev=False
#     ),
# )

auto = ParallelCommandGroup(
    SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),
    
        InstantCommand(lambda: Field.odometry.disable()),
    
        # Shoot first note
        DeployIntake(Robot.intake).withTimeout(0.3),
        PassNote(Robot.wrist),
        
        # Get second note
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
    
        # Shoot second note
        InstantCommand(lambda: Field.odometry.enable()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable()),
    
        # Get third note
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
    
        # Shoot third note
        InstantCommand(lambda: Field.odometry.enable()),
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        InstantCommand(lambda: Field.odometry.disable()),

        PathUntilIntake(path_6, Robot.wrist, Robot.intake, 1)
    
    ),

    # SequentialCommandGroup(
    #     InstantCommand(lambda: Field.odometry.disable()),
    #     path_2,
    #     path_3,
    #     path_4,
    #     path_5,
    #     path_6,
    # )
    # path_far_to_mid,
    # path_mid_to_far

)

routine = AutoRoutine(Pose2d(initial[0], initial[1], math.radians(-120)), auto)