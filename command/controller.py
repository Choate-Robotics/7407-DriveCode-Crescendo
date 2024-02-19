from subsystem import Elevator, Wrist, Intake, Drivetrain, Flywheel
from sensors import FieldOdometry, TrajectoryCalculator

import logging
from command import *
import wpilib, config, constants
import commands2, ntcore
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import WaitUntilCommand, WaitCommand, ParallelCommandGroup, SequentialCommandGroup, InstantCommand, \
    PrintCommand, ParallelDeadlineGroup, RunCommand
from typing import Literal
from units.SI import degrees_to_radians


class Giraffe(commands2.Command):

    def __init__(self, elevator: Elevator, wrist: Wrist, target: config.Giraffe.GiraffePos):
        self.elevator = elevator
        self.wrist = wrist
        self.target = target
        self.finished = False
        self.aiming = False
        self.staging = False
        self.auto_height = False
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('giraffe')

    def finish(self):
        self.finished = True

    def initialize(self):
        self.finished = False
        self.aiming = False
        self.staging = False
        self.auto_height = False

        commands = []
        debug_commands = []

        if self.target.height == None or self.target.wrist_angle == None:
            print('elevator target or wrist target is none')
            self.finished = True
            return  # invalid target

        debug_commands.append(
            PrintCommand("Running both elevator and wrist normally")
        )

        continuous_commands = []

        # if the target wrist angle is 'aim' then set the wrist angle to the calculated angle, we will pass off the aiming command at the end
        if self.target.wrist_angle == config.Giraffe.GiraffePos.Special.kAim:
            continuous_commands.append(
                PrintCommand("Aiming wrist")
            )
            self.aiming = True
            # self.target.wrist_angle = self.wrist.get_wrist_angle()
            self.target.wrist_angle = 20 * degrees_to_radians
            debug_commands.append(
                PrintCommand("Aiming wrist")
            )

        # if the target wrist angle is 'stage' then set the wrist angle to the staging angle, we will pass off the staging command at the end
        if self.target.wrist_angle == config.Giraffe.GiraffePos.Special.kStage:
            self.staging = True
            self.target.wrist_angle = config.staging_angle
            print('staging note to wrist')
            continuous_commands.append(
                FeedIn(self.wrist)
            )
            debug_commands.append(
                PrintCommand("Staging note to wrist")
            )

        if self.target.height == config.Giraffe.GiraffePos.Special.kHeightAuto:
            self.auto_height = True
            self.target.height = self.elevator.get_length()
            continuous_commands.append(
                PrintCommand("Auto height")
            )
            debug_commands.append(
                PrintCommand("Auto height")
            )

        # if the desired elevator height is greater than 0 while the elevator is locked down
        if self.target.height > constants.elevator_max_length_stage and self.elevator.locked or self.target.wrist_angle < constants.wrist_min_rotation_stage and self.elevator.locked:
            self.finished = True
            print('stage is in the way')
            return  # cant perform this action while elevator is locked down

        if not type(self.target.height) == float and not type(self.target.height) == int:
            print('height not float (possibly set wrong enum type)', type(self.target.height))
            return

        if not type(self.target.wrist_angle) == float and not type(self.target.wrist_angle) == int:
            print('wrist not float (possibly set wrong enum type)', type(self.target.wrist_angle))
            return

        commands.append(
            ParallelCommandGroup(
                SetElevator(self.elevator, self.target.height),
                SetWrist(self.wrist, self.target.wrist_angle),
            )
        )

        debug_commands.append(
            PrintCommand('running wrist and elevator normally')
        )

        # commands += continuous_commands

        # print('running alll commands like normal')

        commands2.CommandScheduler.getInstance().schedule(ParallelCommandGroup(
            SequentialCommandGroup(
                *commands,
                InstantCommand(lambda: self.finish()),
                *continuous_commands
            ),
            # SequentialCommandGroup(
            #     *debug_commands
            # )
        )
        )

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted: bool):
        print("GIRAFFE COMMAND FINISHED")
        if interrupted:
            print("GIRAFFE INTERRUPTED")
            self.finished = True


class GiraffeLock(commands2.Command):

    def __init__(self, elevator: Elevator, wrist: Wrist):
        self.elevator = elevator
        self.wrist = wrist
        self.finished = False

    def finish(self):
        self.finished = True

    def initialize(self):
        self.finished = False

        self.elevator.lock()

        self.wrist.lock()

        def wrist_is_over_limit():
            return self.wrist.get_wrist_angle() > constants.wrist_min_rotation_stage

        def elevator_is_over_limit():
            return self.elevator.get_length() > constants.elevator_max_length_stage

        commands = []
        debug_commands = []

        if not elevator_is_over_limit and not wrist_is_over_limit():
            self.finished = True
            return  # already down

        if wrist_is_over_limit():
            commands.append(
                SetWrist(self.wrist, config.wrist_stage_max)
            )
            debug_commands.append(
                PrintCommand("Setting wrist to stage limit")
            )

        if elevator_is_over_limit():
            commands.append(
                SetElevator(self.elevator, config.elevator_stage_max)
            )
            debug_commands.append(
                PrintCommand("Setting elevator to stage limit")
            )

        commands.append(
            InstantCommand(self.finish)
        )

        commands2.CommandScheduler.schedule(ParallelCommandGroup(
            *commands,
            SequentialCommandGroup(
                *debug_commands
            )
        )
        )

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted: bool):
        if interrupted:
            self.finished = True


class StageNote(SequentialCommandGroup):

    def __init__(self, elevator: Elevator, wrist: Wrist, intake: Intake):
        super().__init__(
            # FeedIn(wrist).alongWith(
            Giraffe(elevator, wrist, config.Giraffe.kStage),
            # ),
            PassIntakeNote(intake),
            WaitUntilCommand(wrist.note_detected),
            IntakeIdle(intake),
            Giraffe(elevator, wrist, config.Giraffe.kIdle),
        )


class ShootSpeaker(SequentialCommandGroup):

    def __init__(self, drivetrain: Drivetrain, calculations: TrajectoryCalculator, elevator: Elevator, wrist: Wrist,
                 flywheel: Flywheel, atPose: Pose2d | None = None):
        super().__init__(
            ParallelCommandGroup(
                SetFlywheelLinearVelocity(flywheel, config.v0_flywheel),
                Giraffe(elevator, wrist, config.Giraffe.kAim),
                PrintCommand('Aim Drivetrain')
            ),
            WaitUntilCommand(lambda: elevator.ready_to_shoot and wrist.ready_to_shoot and drivetrain.ready_to_shoot),
            PassNote(wrist),
        )


class ShootAmp(SequentialCommandGroup):

    def __init__(self, drivetrain: Drivetrain, elevator: Elevator, wrist: Wrist, flywheel: Flywheel):
        super().__init__(
            ParallelCommandGroup(
                SetFlywheelLinearVelocity(flywheel, config.flywheel_amp_speed),
                Giraffe(elevator, wrist, config.Giraffe.kAmp),
                PrintCommand('Lock Drivetrain with amp')
            ),
            WaitUntilCommand(lambda: elevator.ready_to_shoot and wrist.ready_to_shoot and drivetrain.ready_to_shoot),
            PassNote(self.wrist),
        )


class EnableClimb(ParallelCommandGroup):

    def __init__(self, elevator: Elevator, wrist: Wrist, intake: Intake):
        super().__init__(
            Giraffe(elevator, wrist, config.Giraffe.kClimbReach),
            SequentialCommandGroup(
                WaitUntilCommand(lambda: wrist.get_wrist_angle() < config.wrist_tent_limit),
                DeployTenting(intake)
            )
        )
