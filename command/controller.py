import logging  # noqa
import math  # noqa
from typing import Literal  # noqa

from subsystem import Elevator, Wrist, Intake, Drivetrain, Flywheel
from sensors import FieldOdometry, TrajectoryCalculator, Limelight
import commands2
import ntcore
import wpilib  # noqa
from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    PrintCommand,
    SequentialCommandGroup,
    WaitCommand,
    ParallelRaceGroup,
    WaitUntilCommand,
    ConditionalCommand,
    ParallelDeadlineGroup
)
from commands2.command import Command
from wpimath.geometry import Pose2d, Rotation2d  # noqa

import config
import constants
from command import (
    AimWrist,
    DeployTenting,
    DriveSwerveAim,
    FeedIn,
    IntakeIdle,
    PassIntakeNote,
    PassNote,
    RunIntakeConstant,
    SetElevator,
    SetElevatorClimbDown,
    SetFlywheelLinearVelocity,
    SetWrist,
    SetWristIdle,
    UnDeployTenting,
    DriveSwerveNoteLineup
)

# from command import *  # noqa
from sensors import FieldOdometry, TrajectoryCalculator  # noqa
from subsystem import Drivetrain, Elevator, Flywheel, Intake, Wrist
from units.SI import degrees_to_radians, inches_to_meters
from wpilib import Joystick


class Giraffe(commands2.Command):
    """
    Giraffe command that sets the elevator and wrist to a certain position.

    :param elevator: Elevator subsystem
    :param wrist: Wrist subsystem
    :param target: config.Giraffe.GiraffePos
    :param shot_calc: TrajectoryCalculator | None

    Runs respective commands with safety and overlap checks.
    """

    def __init__(
            self,
            elevator: Elevator,
            wrist: Wrist,
            target: config.Giraffe.GiraffePos,
            shot_calc: TrajectoryCalculator | None = None,
    ):
        self.elevator = elevator
        self.wrist = wrist
        self.target = target
        self.shot_calc = shot_calc
        self.finished = False
        self.aiming = False
        self.staging = False
        self.auto_height = False
        self.continuous_command: FeedIn | AimWrist | None = None  # noqa
        self.table = ntcore.NetworkTableInstance.getDefault().getTable(
            "giraffe commands"
        )

    def finish(self):
        self.finished = True
        self.table.putBoolean("finished", True)

    def initialize(self):
        self.finished = False
        self.aiming = False
        self.staging = False
        self.auto_height = False
        self.table.putBoolean("finished", False)
        self.table.putBoolean("interrupted", False)
        self.continuous_command: FeedIn | AimWrist | None = None  # noqa

        commands = []
        debug_commands = []

        if self.target.height == None or self.target.wrist_angle == None:  # noqa
            print("elevator target or wrist target is none")
            self.finished = True
            return  # invalid target

        debug_commands.append(PrintCommand("Running both elevator and wrist normally"))

        # if the target wrist angle is 'aim' then set the wrist angle to the calculated angle,
        # we will pass off the aiming command at the end
        if self.target.wrist_angle == config.Giraffe.GiraffePos.Special.kAim:
            if self.shot_calc is None:
                print("no shot calc")
                self.finished = True
                return
            self.continuous_command = AimWrist(self.wrist, self.shot_calc)  # noqa

            self.aiming = True
            self.target.wrist_angle = self.wrist.get_wrist_angle()
            # self.target.wrist_angle = 17.5 * degrees_to_radians
            debug_commands.append(PrintCommand("Aiming wrist"))

        # if the target wrist angle is 'stage' then set the wrist angle to
        # the staging angle, we will pass off the staging command at the end
        if self.target.wrist_angle == config.Giraffe.GiraffePos.Special.kStage:
            self.staging = True
            self.target.wrist_angle = config.staging_angle
            print("staging note to wrist")
            self.continuous_command = FeedIn(self.wrist)  # noqa
            # InstantCommand(lambda: self.wrist.feed_in())
            debug_commands.append(PrintCommand("Staging note to wrist"))

        if self.target.wrist_angle == config.Giraffe.GiraffePos.Special.kCurrentAngle:
            self.target.wrist_angle = self.wrist.get_wrist_angle()
            debug_commands.append(PrintCommand("Setting wrist to current angle"))

        if self.target.height == config.Giraffe.GiraffePos.Special.kHeightAuto:
            self.auto_height = True
            self.target.height = self.elevator.get_length()
            debug_commands.append(PrintCommand("Auto height"))

        if self.target.height == config.Giraffe.GiraffePos.Special.kCurrentHeight:
            self.target.height = self.elevator.get_length()
            debug_commands.append(PrintCommand("Setting elevator to current height"))

        # if the desired elevator height is greater than 0 while the elevator is locked down
        if (
                self.target.height > constants.elevator_max_length_stage
                and self.elevator.locked
                or self.target.wrist_angle < constants.wrist_min_rotation_stage
                and self.elevator.locked
        ):
            self.finished = True
            print("stage is in the way")
            return  # cant perform this action while elevator is locked down

        if (
                type(self.target.height) is not float
                and type(self.target.height) is not int
        ):
            print(
                "height not float (possibly set wrong enum type)",
                type(self.target.height),
            )
            return

        if (
                type(self.target.wrist_angle) is not float  # noqa
                and type(self.target.wrist_angle) is not int  # noqa
        ):
            print(
                "wrist not float (possibly set wrong enum type)",
                type(self.target.wrist_angle),
            )
            return

        commands.append(
            ParallelCommandGroup(
                SetElevator(self.elevator, self.target.height),  # noqa
                SetWrist(self.wrist, self.target.wrist_angle),  # noqa
            )
        )

        debug_commands.append(PrintCommand("running wrist and elevator normally"))

        # print('running alll commands like normal')

        commands2.CommandScheduler.getInstance().schedule(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    *commands,
                    InstantCommand(lambda: self.finish()),
                    # ParallelCommandGroup(
                    #     InstantCommand(lambda: self.finish()),
                    #     self.continuous_command
                    # )
                ),
                # self.continuous_command
            )
        )

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted: bool):
        print("GIRAFFE COMMAND FINISHED")
        if interrupted:
            self.finished = False
            self.table.putBoolean("interrupted", True)
            # raise NotImplementedError
            # self.finished = True
        else:
            self.table.putBoolean("finished", False)
        self.finished = False
        commands2.CommandScheduler.getInstance().schedule(self.continuous_command)


class StageNote(SequentialCommandGroup):
    """
    Stages a note to the feeder from the intake

    Args:
        SequentialCommandGroup (elevator): Elevator subsystem
        SequentialCommandGroup (wrist): Wrist subsystem
        SequentialCommandGroup (intake): Intake subsystem
    """

    def __init__(self, elevator: Elevator, wrist: Wrist, intake: Intake):
        super().__init__(
            ParallelCommandGroup(
                FeedIn(wrist),  # noqa
                # Giraffe(elevator, wrist, config.Giraffe.kStage),
                PassIntakeNote(intake),  # noqa
            ),
            IntakeIdle(intake),  # noqa
        )


class IntakeStageNote(SequentialCommandGroup):
    def __init__(self, wrist: Wrist, intake: Intake):
        super().__init__(
            SetWristIdle(wrist), RunIntakeConstant(intake), FeedIn(wrist)  # noqa
        )  # noqa


class IntakeStageNoteAuto(ParallelRaceGroup):
    def __init__(self, wrist: Wrist, intake: Intake):
        super().__init__(
            IntakeStageNote(wrist, intake),
            WaitUntilCommand(lambda: wrist.note_in_feeder()),
        )


class PathUntilIntake(ParallelRaceGroup):
    def __init__(self, path: Command, wrist: Wrist, intake: Intake,
                 waittime: float = config.auto_path_intake_note_deadline):
        super().__init__(
            SequentialCommandGroup(
                path,
                WaitCommand(waittime),
                WaitUntilCommand(lambda: not wrist.note_in_feeder())
            ),
            IntakeStageNote(wrist, intake)
        )

class PathIntakeAim(ParallelRaceGroup):
    def __init__(self, path: Command, wrist: Wrist, intake: Intake, calculations: TrajectoryCalculator):
        super().__init__(
            path,
            SequentialCommandGroup(
                IntakeStageNote(wrist, intake),
                AimWrist(wrist, calculations)
            ),
        )

class IntakeStageIdle(SequentialCommandGroup):
    def __init__(self, wrist: Wrist, intake: Intake):
        super().__init__(
            IntakeIdle(intake), InstantCommand(lambda: wrist.stop_feed())
        )  # noqa


class AimWristSpeaker(ParallelCommandGroup):
    """
    Aims the drivetrain, elevator, and wrist to shoot a note

    Args:
        SequentialCommandGroup (calculations): TrajectoryCalculator
        SequentialCommandGroup (elevator): Elevator subsystem
        SequentialCommandGroup (wrist): Wrist subsystem
        SequentialCommandGroup (flywheel): Flywheel subsystem
    """

    def __init__(
            self,
            calculations: TrajectoryCalculator,
            elevator: Elevator,
            wrist: Wrist,
            flywheel: Flywheel,
            atPose: Pose2d | None = None,
    ):
        super().__init__(
            SetFlywheelLinearVelocity(flywheel, config.v0_flywheel),  # noqa
            Giraffe(elevator, wrist, config.Giraffe.kAim, calculations),
        )


class Shoot(SequentialCommandGroup):
    """
    Shoots a note

    Args:
        SequentialCommandGroup (wrist): Wrist subsystem
    """

    def __init__(self, wrist: Wrist):
        super().__init__(
            PassNote(wrist),  # noqa
            WaitCommand(0.5),
            SetWristIdle(wrist),
        )
        self.InterruptionBehavior = 1


class ShootAuto(SequentialCommandGroup):
    """Shoots a note during the autonomous period

    Args:
        SequentialCommandGroup (drivetrain): Drivetrain subsystem
        SequentialCommandGroup (wrist): Wrist subsystem
        SequentialCommandGroup (flywheel): Flywheel subsystem
        SequentialCommandGroup (calculations): TrajectoryCalculator
    """

    def __init__(
            self,
            drivetrain: Drivetrain,
            wrist: Wrist,
            flywheel: Flywheel,
            traj_cal: TrajectoryCalculator,
    ):
        super().__init__(
            ParallelCommandGroup(  # Aim
                AimWrist(wrist, traj_cal),  # noqa
                DriveSwerveAim(drivetrain, traj_cal, limit_speed=False, auto=True),  # noqa
            )
            .until(
                lambda: drivetrain.ready_to_shoot
                        and wrist.ready_to_shoot
                        and flywheel.ready_to_shoot
            )
            .withTimeout(config.auto_shoot_deadline),
            PassNote(wrist),  # noqa
        )


class EnableClimb(SequentialCommandGroup):
    """
    Raises the elevator and wrist and deploys tenting to prepare for climb
    """

    def __init__(self, elevator: Elevator, wrist: Wrist, intake: Intake):
        super().__init__(
            ParallelCommandGroup(
                SetWrist(wrist, -44 * degrees_to_radians),  # noqa
                SetElevator(elevator, config.Giraffe.kClimbReach.height / 3),  # noqa
                SequentialCommandGroup(
                    WaitUntilCommand(lambda: wrist.get_wrist_angle() < 20 * degrees_to_radians),  # noqa
                    DeployTenting(intake),
                )  # noqa
            ),
            SetWrist(wrist, 35 * degrees_to_radians),  # noqa
            ParallelCommandGroup(
                SetElevator(elevator, config.Giraffe.kClimbReach.height),  # noqa
                SetWrist(wrist, 25 * degrees_to_radians),  # noqa
            ),
        )


class ClimbDown(ParallelCommandGroup):
    def __init__(self, elevator: Elevator, wrist: Wrist):
        super().__init__(
            SetElevatorClimbDown(elevator),
            SetWrist(wrist, config.Giraffe.kClimbPullUp.wrist_angle),  # noqa
        )


class UndoClimb(ParallelCommandGroup):
    """
    Undeploys tenting and lowers elevator
    """

    def __init__(self, elevator: Elevator, wrist: Wrist, intake: Intake):
        super().__init__(
            UnDeployTenting(intake),  # noqa
            SetElevator(elevator, config.Giraffe.kIdle.height),  # noqa
            SetWristIdle(wrist),  # noqa
        )


class ScoreTrap(SequentialCommandGroup):
    """
    Raises elevator and then feeds note out to score in trap
    """

    def __init__(self, elevator: Elevator, wrist: Wrist):
        super().__init__(
            # Giraffe(elevator, wrist, config.Giraffe.kClimbTrap),
            SetWrist(wrist, -25 * degrees_to_radians),
            SetElevator(  # noqa
                elevator, constants.elevator_max_length - (4 * inches_to_meters)
            ),
            # InstantCommand(lambda: wrist.feed_out())
            ParallelCommandGroup(
                SetWrist(wrist, 0),  # noqa
                SetElevator(  # noqa
                    elevator, constants.elevator_max_length - (1 * inches_to_meters)
                ),
            )
            .withTimeout(2)
            .andThen(InstantCommand(lambda: wrist.feed_out())),
            # FeedOut(wrist)
        )


class Amp(ParallelCommandGroup):
    def __init__(self, elevator: Elevator, wrist: Wrist):
        super().__init__(
            SetWrist(wrist, config.Giraffe.kAmp.wrist_angle),
            SetElevator(elevator, config.Giraffe.kAmp.height),
            # SetFlywheelVelocityIndependent(flywheel, (config.flywheel_amp_speed, config.flywheel_amp_speed/4))
        )


class EmergencyManuver(SequentialCommandGroup):
    def __init__(self, wrist: Wrist, intake: Intake):
        super().__init__(
            ParallelCommandGroup(
                PassIntakeNote(intake),
                SetWrist(wrist, -10 * degrees_to_radians),  # noqa
            ),
            DeployTenting(intake),
            UnDeployTenting(intake),
            SetWrist(wrist, config.staging_angle)
        )


class AutoPickupNote(SequentialCommandGroup):

    def __init__(self, drivetrain: Drivetrain, wrist: Wrist, intake: Intake, limelight: Limelight):
        super().__init__(
            ConditionalCommand(
                SequentialCommandGroup(
                    IntakeStageNote(wrist, intake),
                    IntakeStageIdle(wrist, intake)
                ),
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        IntakeStageNote(wrist, intake),
                        ParallelDeadlineGroup(
                            WaitUntilCommand(lambda: intake.detect_note()),
                            DriveSwerveNoteLineup(drivetrain, limelight)
                        )
                    )
                ),
                lambda: intake.detect_note() or wrist.detect_note_first() or wrist.detect_note_second()
            )
        )


class ControllerRumble(InstantCommand):

    def __init__(self, controller: Joystick, intensity: float = 1.0):
        super().__init__(lambda: controller.setRumble(
            controller.RumbleType.kBothRumble,
            intensity),
                         )


class ControllerRumbleTimeout(SequentialCommandGroup):

    def __init__(self, controller: Joystick, timeout: float = 3, intensity: float = 1.0):
        super().__init__(
            ControllerRumble(controller, intensity),
            WaitCommand(timeout),
            ControllerRumble(controller, 0)
        )
