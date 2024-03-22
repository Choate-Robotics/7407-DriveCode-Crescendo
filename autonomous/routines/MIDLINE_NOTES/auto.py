from command.autonomous.custom_pathing import FollowPathCustom, AngleType
from command.autonomous.trajectory import CustomTrajectory, PoseType
import command
from robot_systems import Robot, Field
from utils import POIPose
import config
import math
from command import *

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
from autonomous.routines.MIDLINE_NOTES.coords import *

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*shoot_first_note[0])),
        waypoints=[coord for coord in shoot_first_note[1]],
        end_pose=shoot_first_note[2],
        max_velocity=5,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=AngleType.calculate
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=get_second_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in get_second_note[1]],
        end_pose=get_second_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=math.radians(-180)
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=shoot_second_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_second_note[1]],
        end_pose=shoot_second_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False
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
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=math.radians(147)
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        # start_pose=shoot_third_note[0],
        start_pose=PoseType.current,
        waypoints=[coord for coord in shoot_third_note[1]],
        end_pose=shoot_third_note[2],
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False
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
        max_velocity=6,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=True
    ),
    theta_f=math.radians(-180)
)

auto = ParallelCommandGroup(
    SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
    SequentialCommandGroup(
        ZeroWrist(Robot.wrist),
        ZeroElevator(Robot.elevator),

        ParallelCommandGroup(
            path_1,
            DeployIntake(Robot.intake),
        ),

        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        # ParallelCommandGroup(
        #     path_2,
        #     SequentialCommandGroup(
        #         SetWristIdle(Robot.wrist),
        #         ParallelRaceGroup(
        #             IntakeStageNote(Robot.wrist, Robot.intake)
        #         )
        #         .withInterrupt(lambda: Robot.wrist.note_in_feeder()).withTimeout(4)
        #     )
        # ),
        # path_2,
        # ParallelRaceGroup(
        #     SequentialCommandGroup(
        #         path_2,
        #         WaitCommand(config.auto_path_intake_note_deadline)
        #     ),
        #     IntakeStageNote(Robot.wrist, Robot.intake)
        # ),
        PathUntilIntake(path_2, Robot.wrist, Robot.intake),
        path_3,
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        # ParallelCommandGroup(
        #     path_4,
        #     SequentialCommandGroup(
        #         SetWristIdle(Robot.wrist),
        #         IntakeStageNote(Robot.wrist, Robot.intake).withTimeout(4)
        #     )
        # ),
        PathUntilIntake(path_4, Robot.wrist, Robot.intake),
        path_5,
        ShootAuto(Robot.drivetrain, Robot.wrist, Robot.flywheel, Field.calculations),
        # ParallelCommandGroup(
        #     path_6,
        #     SequentialCommandGroup(
        #         SetWristIdle(Robot.wrist),
        #         IntakeStageNote(Robot.wrist, Robot.intake)
        #     )
        # ),
        PathUntilIntake(path_6, Robot.wrist, Robot.intake)
    )
    # SequentialCommandGroup(
    #     path_1,
    #     WaitCommand(1),
    #     path_2,
    #     WaitCommand(1),
    #     path_3,
    #     WaitCommand(1),
    #     path_4,
    #     WaitCommand(1),
    #     path_5,
    #     WaitCommand(1),
    #     path_6
    # )
)

routine = AutoRoutine(Pose2d(*initial), auto)
