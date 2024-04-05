from command.autonomous.custom_pathing import FollowPathCustom, AngleType
from command.autonomous.trajectory import CustomTrajectory, PoseType
from robot_systems import Robot, Field, Sensors
from utils import POIPose
from command import *
import config
import math

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
from autonomous.routines.FOUR_NOTE_MIDDLE.coords import initial


auto = SequentialCommandGroup(
    SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),

        # Shoot preload
        PassNote(Robot.wrist),

        SetWristIdle(Robot.wrist)
    )
)

routine = AutoRoutine(Pose2d(*initial), auto)
