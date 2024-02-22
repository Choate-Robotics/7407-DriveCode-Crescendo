from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot, Field
from utils import POIPose
from command import *
import config

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
    WaitCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.FOUR_NOTE.coords import (
    get_first_note,
    get_second_note,
    get_third_note,
    initial
)

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_first_note[0])),
        waypoints=[Translation2d(*coord) for coord in get_first_note[1]],
        end_pose=get_first_note[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=True
    )
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_second_note[0],
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=False
    )
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_third_note[0],
        waypoints=[coord for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=False
    )
)

auto = SequentialCommandGroup(
    ZeroWrist(Robot.wrist),
    ZeroElevator(Robot.elevator),
    # SetFlywheelLinearVelocity(Robot.flywheel, config.v0_flywheel), #spin up flywheels
    # ParallelCommandGroup( #aim
    #     AimWrist(Robot.wrist, Field.calculations),
    #     DriveSwerveAim(Robot.drivetrain, Field.calculations),
    # ).until(lambda: Robot.wrist.ready_to_shoot and Robot.drivetrain.ready_to_shoot and Robot.flywheel.ready_to_shoot),
    # PassNote(Robot.wrist), #shoot preload
    
    ParallelCommandGroup( #shoot preload and deploy intake
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        DeployIntake(Robot.intake)
    ),
    
    ParallelCommandGroup( #go to and collect first note
        path_1,
        RunIntake(Robot.intake)
    ),
    ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations), #shoot note one

    ParallelCommandGroup( #go to and collect note 2
        path_2,
        RunIntake(Robot.intake)
    ),
    ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations), #shoot note two
    
    ParallelCommandGroup( #go to and collect note 3
        path_3,
        RunIntake(Robot.intake)
    ),
    ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations), #shoot note three
    

    # path_1,
    # path_2,
    # path_3
)

routine = AutoRoutine(Pose2d(*initial), auto)