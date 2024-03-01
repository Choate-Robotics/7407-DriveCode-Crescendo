from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
import command
from robot_systems import Robot, Field
from utils import POIPose
import config

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.LEFT_LEAVE.coords import (
    initial,
    leave,
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*leave[0])),
        waypoints=[Translation2d(*coord) for coord in leave[1]],
        end_pose=leave[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)


auto = SequentialCommandGroup(
    command.DrivetrainZero(Robot.drivetrain),
    command.ZeroWrist(Robot.wrist),
    command.ZeroElevator(Robot.elevator),
    
    ParallelCommandGroup(
        command.ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        command.DeployIntake(Robot.intake)
    ),

    path_1,

    
)

routine = AutoRoutine(Pose2d(*initial), auto)
