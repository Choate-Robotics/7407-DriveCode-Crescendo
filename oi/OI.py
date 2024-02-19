from utils import LocalLogger

import commands2
import command, config, constants
from robot_systems import Robot, Sensors, Field
from oi.keymap import Keymap

log = LocalLogger("OI")


class OI:

    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")

        Keymap.Drivetrain.RESET_GYRO.onTrue(
            command.DrivetrainZero(Robot.drivetrain))\
            .onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Shooter.AIM.whileTrue(
            command.DriveSwerveAim(Robot.drivetrain, Field.calculations)
        ).onFalse(
            command.DriveSwerveCustom(Robot.drivetrain)
        )
        
        Keymap.Intake.INTAKE_IN.and_(lambda: not Robot.intake.note_in_intake).onTrue(
            command.RunIntake(Robot.intake)
        ).onFalse(
            command.IntakeIdle(Robot.intake)
        )
        
        Keymap.Intake.INTAKE_OUT.onTrue(
            command.EjectIntake(Robot.intake)
        ).onFalse(
            command.IntakeIdle(Robot.intake)
        )
        
        Keymap.Elevator.ELEVATOR_HIGH.onTrue(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kElevatorHigh)
        )
        
        Keymap.Elevator.ELEVATOR_MID.onTrue(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kElevatorMid)
        )
        
        Keymap.Elevator.ELEVATOR_LOW.onTrue(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kElevatorLow)
        )
        
        Keymap.Elevator.AMP.onTrue(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAmp)
        ).onFalse(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kIdle)
        )
        
        Keymap.Feeder.CLEAR_NOTE.onTrue(
                command.PassNote(Robot.wrist).withTimeout(2).andThen(commands2.WaitCommand(2).andThen(
                    commands2.InstantCommand(lambda: Robot.wrist.stop_feed())
                ).andThen(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kIdle)).alongWith(
                    commands2.InstantCommand(lambda: Robot.wrist.set_note_not_staged())
                ))
            )
        
        Keymap.Climb.CLIMB_UP.toggleOnTrue(
            command.EnableClimb(Robot.elevator, Robot.wrist, Robot.intake)
        )\
        #     .toggleOnFalse(
        #     command.UndoClimb(Robot.elevator, Robot.wrist, Robot.intake)
        # )
        
        Keymap.Climb.CLIMB_DOWN.onTrue(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kClimbPullUp)
        )
