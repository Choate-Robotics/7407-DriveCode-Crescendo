from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from command import DrivetrainZero, Giraffe, RunIntake, PassNote, SetFlywheelLinearVelocity, DriveSwerveAim, FeedIn, PassIntakeNote
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
from autonomous.routines.AIM_SHOOT.coords import (
    initial,
    shooting_position
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*shooting_position[0])),
        waypoints=[Translation2d(*coord) for coord in shooting_position[1]],
        end_pose=shooting_position[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)


# Between paths, need to score rings
auto = SequentialCommandGroup(
    DrivetrainZero(Robot.drivetrain),
    ParallelCommandGroup(
        SetFlywheelLinearVelocity(Robot.flywheel, 5),
        SequentialCommandGroup(
            # DriveSwerveAim(Robot.drivetrain),
            # Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim), # aim
            # PassNote(Robot.wrist), # shoot
            path_1,
            # WaitCommand(0.5),
            # Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kStage),
            # ParallelCommandGroup(
            #     FeedIn(Robot.wrist),
            #     PassIntakeNote(Robot.intake),
            # ),
            # RunIntake(Robot.intake), # intake
           
            InstantCommand(lambda: print("Done")),
        )
    )
)

routine = AutoRoutine(Pose2d(*initial), auto)
