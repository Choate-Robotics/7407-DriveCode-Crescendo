import math
from autonomous.auto_routine import AutoRoutine
from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from wpimath.geometry import Pose2d, Translation2d

from coords import start_rotating, blue_team
from command.autonomous.custom_pathing import RotateInPlace
from command.autonomous.trajectory import CustomTrajectory

from robot_systems import Robot, Sensors
from units.SI import meters_per_second, meters_per_second_squared

max_vel: meters_per_second = 3
max_accel: meters_per_second_squared = 2


path_1 = RotateInPlace(
    subsystem=Robot.drivetrain,
    theta_f=3.14
)

auto = SequentialCommandGroup(
    path_1,
    InstantCommand(lambda: print("Done")),
)

routine = AutoRoutine(Pose2d(*start_rotating.initial), auto, blue_team=blue_team)