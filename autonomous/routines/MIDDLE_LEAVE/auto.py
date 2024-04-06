import constants
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
from autonomous.routines.MIDDLE_LEAVE.coords import (
    initial,
    leave,
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*leave[0])),
        waypoints=[Translation2d(*coord) for coord in leave[1]],
        end_pose=POIPose(Pose2d(*leave[2])),
        max_velocity=config.drivetrain_max_vel_auto,
        max_accel=config.drivetrain_max_accel_auto - 0.75,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=0.03,
)


auto = ParallelCommandGroup(
    command.SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        command.ZeroWrist(Robot.wrist),
        command.ZeroElevator(Robot.elevator),

        command.PassNote(Robot.wrist),

        WaitCommand(10),

        path_1,
    ),
)

routine = AutoRoutine(Pose2d(*initial), auto)
