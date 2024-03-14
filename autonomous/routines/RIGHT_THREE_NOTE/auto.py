from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
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
from autonomous.routines.RIGHT_THREE_NOTE.coords import (
    get_first_note,
    initial
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_first_note[0])),
        waypoints=[coord for coord in get_first_note[1]],
        end_pose=get_first_note[2],
        max_velocity=7,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

auto = ParallelCommandGroup(
    SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),

        # Shoot preload and deploy intake
        ParallelCommandGroup(
            ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
            DeployIntake(Robot.intake)
        ),

        # Reset drivetrain
        ParallelCommandGroup(
            DriveSwerveHoldRotation(Robot.drivetrain, math.radians(-180)),
            SetWristIdle(Robot.wrist),
        ),

        # Get second note
        ParallelCommandGroup(
            path_1,
            IntakeStageNote(Robot.wrist, Robot.intake)
        ),
    )
)

routine = AutoRoutine(Pose2d(*initial), auto)