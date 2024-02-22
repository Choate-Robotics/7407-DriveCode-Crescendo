import command
import config
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from command import Giraffe

from robot_systems import Robot, Field
from utils import POIPose

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    WaitCommand,
    ParallelCommandGroup,
    ParallelRaceGroup,
    PrintCommand,
    WaitUntilCommand
)

from autonomous.auto_routine import AutoRoutine
from autonomous.routines.FIVE_NOTE.coords import *

from wpimath.geometry import Pose2d, Translation2d

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=POIPose(Pose2d(*get_second_note[0])),
        waypoints=[Translation2d(*coord) for coord in get_second_note[1]],
        end_pose=get_second_note[2],
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
        start_pose=get_third_note[0],
        waypoints=[Translation2d(*coord) for coord in get_third_note[1]],
        end_pose=get_third_note[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=get_fourth_note[0],
        waypoints=[Translation2d(*coord) for coord in get_fourth_note[1]],
        end_pose=get_fourth_note[2],
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
        start_pose=go_to_midline_1[0],
        waypoints=[Translation2d(*coord) for coord in go_to_midline_1[1]],
        end_pose=go_to_midline_1[2],
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
        start_pose=get_fifth_note[0],
        waypoints=[Translation2d(*coord) for coord in get_fifth_note[1]],
        end_pose=get_fifth_note[2],
        max_velocity=12,
        max_accel=3,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=0.03,
)

path_6 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=go_to_midline_2[0],
        waypoints=[Translation2d(*coord) for coord in go_to_midline_2[1]],
        end_pose=go_to_midline_2[2],
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
    path_1,
    path_2,
    path_3,
    path_4,
    path_5,
    path_6,

    # command.ZeroElevator(Robot.elevator),
    # command.ZeroWrist(Robot.wrist),
    # ParallelRaceGroup(
    #     command.SetFlywheelLinearVelocity(Robot.flywheel, config.v0_flywheel),
    #     SequentialCommandGroup(
    #         InstantCommand(lambda: print("Entered Sequential Command")),
            # Shoot first note
            # SequentialCommandGroup(
                # Stage note to feeder from intake
                # Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kStage),
                # ParallelCommandGroup(
                #     command.FeedIn(Robot.wrist),
                #     command.PassIntakeNote(Robot.intake),
                # ),
                #
                # # Aim wrist
                # Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim, Field.calculations),

                # Shoot note
                # ParallelRaceGroup(
                #     InstantCommand(lambda: print("Passing Note to flywheel")),
                    # Passes note to flywheel
                    # command.PassNote(Robot.wrist),
                    # WaitUntilCommand(Robot.flywheel.note_shot)
                # ),

                # PrintCommand("Shot first note"),
            # ),

            # # Go to second note
            # path_1,
            #
            # # Intake second note
            # command.RunIntake(Robot.intake),
            # InstantCommand(lambda: print("Intaked second note")),
            #
            # # Shoot second note
            # SequentialCommandGroup(
            #     Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kStage),
            #     ParallelCommandGroup(
            #         command.FeedIn(Robot.wrist),
            #         command.PassIntakeNote(Robot.intake),
            #     ),
            #     Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim, Field.calculations),
            #     ParallelRaceGroup(
            #         command.PassNote(Robot.wrist),
            #         WaitUntilCommand(Robot.flywheel.note_shot)
            #     ),
            #     PrintCommand("Shot second note"),
            # ),
            #
            # # Go to third note
            # path_2,
            #
            # # Intake third note
            # command.RunIntake(Robot.intake),
            # InstantCommand(lambda: print("Intaked third note")),
            #
            # # Shoot third note
            # SequentialCommandGroup(
            #     Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kStage),
            #     ParallelCommandGroup(
            #         command.FeedIn(Robot.wrist),
            #         command.PassIntakeNote(Robot.intake),
            #     ),
            #     Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim, Field.calculations),
            #     ParallelRaceGroup(
            #         command.PassNote(Robot.wrist),
            #         WaitUntilCommand(Robot.flywheel.note_shot)
            #     ),
            #     PrintCommand("Shot third note"),
            # ),
            #
            # # Go to fourth note
            # path_3,
            #
            # # Intake fourth note
            # command.RunIntake(Robot.intake),
            # InstantCommand(lambda: print("Intaked fourth note")),
            #
            # # Go to midline for fourth note
            # path_4,
            # InstantCommand(lambda: print("Went to midline")),
            #
            # # Shoot fourth note
            # SequentialCommandGroup(
            #     Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kStage),
            #     ParallelCommandGroup(
            #         command.FeedIn(Robot.wrist),
            #         command.PassIntakeNote(Robot.intake),
            #     ),
            #     Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim, Field.calculations),
            #     ParallelRaceGroup(
            #         command.PassNote(Robot.wrist),
            #         WaitUntilCommand(Robot.flywheel.note_shot)
            #     ),
            #     PrintCommand("Shot fourth note"),
            # ),
            #
            # # Go to fifth note
            # path_5,
            #
            # # Intake fifth note
            # command.RunIntake(Robot.intake),
            # InstantCommand(lambda: print("Intaked fifth note")),
            #
            # # Go to midline for fifth note
            # path_6,
            #
            # # Shoot fifth note
            # SequentialCommandGroup(
            #     Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kStage),
            #     ParallelCommandGroup(
            #         command.FeedIn(Robot.wrist),
            #         command.PassIntakeNote(Robot.intake),
            #     ),
            #     Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim, Field.calculations),
            #     ParallelRaceGroup(
            #         command.PassNote(Robot.wrist),
            #         WaitUntilCommand(Robot.flywheel.note_shot)
            #     ),
            #     PrintCommand("Shot fifth note"),
            # ),
        # ),

    # )
)

routine = AutoRoutine(Pose2d(*initial), auto)
