from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from command import DrivetrainZero, Giraffe, RunIntake, PassNote, SetFlywheelLinearVelocity
from robot_systems import Robot
from utils import POIPose
import config

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.AMP_TWO_PIECE.coords import (
    initial,
    amp_1,
    get_first_note,
    amp_2,
    get_second_note,
    shoot_second_note,
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*amp_1[0])),
        waypoints=[Translation2d(*coord) for coord in amp_1[1]],
        end_pose=amp_1[2],
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
        start_pose=get_first_note[0],
        waypoints=[Translation2d(*coord) for coord in get_first_note[1]],
        end_pose=get_first_note[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=amp_2[0],
        waypoints=[Translation2d(*coord) for coord in amp_2[1]],
        end_pose=amp_2[2],
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
        start_pose=get_second_note[0],
        waypoints=[Translation2d(*coord) for coord in get_second_note[1]],
        end_pose=get_second_note[2],
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
        start_pose=shoot_second_note[0],
        waypoints=[Translation2d(*coord) for coord in shoot_second_note[1]],
        end_pose=shoot_second_note[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)


# Between paths, need to score rings
auto = SequentialCommandGroup(
    DrivetrainZero(Robot.drivetrain),
    ParallelCommandGroup(
        SetFlywheelLinearVelocity(Robot.flywheel, 2),
        SequentialCommandGroup(
            path_1,
            Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAmp), # aim
            PassNote(Robot.wrist), # shoot
            path_2,
            RunIntake(Robot.intake), # intake
            path_3,
            Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAmp), # aim
            PassNote(Robot.wrist), # shoot
            path_4,
            RunIntake(Robot.intake), # intake
            path_5,
            Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAmp), # aim
            PassNote(Robot.wrist), # shoot
            
            InstantCommand(lambda: print("Done")),
        )
        
    )
    
)

routine = AutoRoutine(Pose2d(*initial), auto)
