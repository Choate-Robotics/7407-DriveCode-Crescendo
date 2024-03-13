from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
import command
from robot_systems import Robot, Field
from utils import POIPose
import config
import math

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.MIDLINE_NOTES.coords import (
    initial,
    come_out_shoot_preload,
    get_first_ring,
    come_back_to_shoot_first_ring,
    get_second_ring,
    come_back_to_shoot_second_ring,
    # get_third_ring,
    # come_back_to_shoot_third_ring,
)

from wpimath.geometry import Pose2d, Translation2d

path_0 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*come_out_shoot_preload[0])),
        waypoints=[Translation2d(*coord) for coord in come_out_shoot_preload[1]],
        end_pose=POIPose(Pose2d(*come_out_shoot_preload[2])),
        max_velocity=5,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    # period=0.03,
)

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_first_ring[0])),
        waypoints=[Translation2d(*coord) for coord in get_first_ring[1]],
        end_pose=get_first_ring[2],
        max_velocity=5,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    # period=0.03,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=come_back_to_shoot_first_ring[0],
        waypoints=[Translation2d(*coord) for coord in come_back_to_shoot_first_ring[1]],
        end_pose=come_back_to_shoot_first_ring[2],
        max_velocity=5,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    # period=0.03,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_second_ring[0],
        waypoints=[Translation2d(*coord) for coord in get_second_ring[1]],
        end_pose=get_second_ring[2],
        max_velocity=5,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    # period=0.03,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=come_back_to_shoot_second_ring[0],
        waypoints=[Translation2d(*coord) for coord in come_back_to_shoot_second_ring[1]],
        end_pose=come_back_to_shoot_second_ring[2],
        max_velocity=5,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    # period=0.03,
)

# path_5 = FollowPathCustom(
#     subsystem=Robot.drivetrain,
#     trajectory=CustomTrajectory(
#         start_pose=get_third_ring[0],
#         waypoints=[Translation2d(*coord) for coord in get_third_ring[1]],
#         end_pose=get_third_ring[2],
#         max_velocity=12,
#         max_accel=3,
#         start_velocity=0,
#         end_velocity=0,
#         rev=True,
#     ),
#     period=0.03,
# )

# path_6 = FollowPathCustom(
#     subsystem=Robot.drivetrain,
#     trajectory=CustomTrajectory(
#         start_pose=come_back_to_shoot_third_ring[0],
#         waypoints=[Translation2d(*coord) for coord in come_back_to_shoot_third_ring[1]],
#         end_pose=come_back_to_shoot_third_ring[2],
#         max_velocity=12,
#         max_accel=3,
#         start_velocity=0,
#         end_velocity=0,
#         rev=False,
#     ),
#     period=0.03,
# )

# Between paths, need to score rings
auto = ParallelCommandGroup(
    command.SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        command.ZeroWrist(Robot.wrist),
        command.ZeroElevator(Robot.elevator),
        
        path_0,

        ParallelCommandGroup(
            command.ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
            command.DeployIntake(Robot.intake)
        ),

        ParallelCommandGroup(
            command.DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)),
            command.SetWristIdle(Robot.wrist)
        ),

        # ParallelCommandGroup(
            path_1,
        #     command.RunIntake(Robot.intake)
        # ),

        path_2,

        # command.ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

        # ParallelCommandGroup(
            path_3,
        #     command.RunIntake(Robot.intake)
        # ),

        path_4,

        # command.ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

        # ParallelCommandGroup(
            # path_5,
            # command.RunIntake(Robot.intake)
        # ),

        # path_6,
        # command.ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),

                # InstantCommand(lambda: print("Done")),
            # )
    )
)

routine = AutoRoutine(Pose2d(*initial), auto)
