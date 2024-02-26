from utils import LocalLogger

import commands2
import command, config, constants
from robot_systems import Robot, Sensors, Field
from oi.keymap import Keymap
from math import radians

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
        
        # Keymap.Intake.INTAKE_IN.and_(lambda: not Robot.intake.note_in_intake and not Robot.wrist.note_staged).onTrue(
        #     command.RunIntake(Robot.intake).alongWith(commands2.InstantCommand(lambda: Robot.wrist.feed_idle()))
        # ).onFalse(
        #     command.IntakeIdle(Robot.intake).andThen(commands2.InstantCommand(lambda: Robot.wrist.stop_feed()).onlyIf(lambda: not Robot.intake.note_in_intake))
        # )
        
        Keymap.Intake.INTAKE_IN.onTrue(
            command.IntakeStageNote(Robot.wrist, Robot.intake)
        ).onFalse(
            command.IntakeStageIdle(Robot.wrist, Robot.intake)
        )
        
        Keymap.Intake.INTAKE_OUT.onTrue(
            command.EjectIntake(Robot.intake)
        ).onFalse(
            command.IntakeIdle(Robot.intake)
        )
        
        # Keymap.Elevator.ELEVATOR_HIGH.onTrue(
        #     # command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kElevatorHigh)
        #     command.EmergencyManuver(Robot.wrist, Robot.intake)
        # )
        
        # Keymap.Elevator.ELEVATOR_MID.onTrue(
        #     command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kElevatorMid)
        # )
        
        # Keymap.Elevator.ELEVATOR_LOW.onTrue(
        #     command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kElevatorLow)
        # )
        
        Keymap.Elevator.AMP.onTrue(
            # command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAmp).andThen(command.SetWrist(Robot.wrist, radians(-30)))
            command.Amp(Robot.elevator, Robot.wrist, Robot.flywheel)
        ).onFalse(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kIdle)
        )
        
        Keymap.Feeder.DUMP_NOTE.onTrue(
            command.PassNote(Robot.wrist),
            # command.SetWrist(Robot.wrist, radians(config.staging_angle))
            )
        
        # Keymap.Feeder.CLEAR_NOTE.onTrue(
        #     commands2.InstantCommand(lambda: Robot.wrist.set_note_not_staged())
        # )
        
        Keymap.Feeder.FEED.and_(lambda: not Robot.wrist.note_detected()).onTrue(
            commands2.InstantCommand(lambda: Robot.wrist.feed_in())
        ).onFalse(
            commands2.InstantCommand(lambda: Robot.wrist.stop_feed())
        )
        
        Keymap.Feeder.UNFEED.onTrue(
            commands2.InstantCommand(lambda: Robot.wrist.feed_out())
        ).onFalse(
            commands2.InstantCommand(lambda: Robot.wrist.stop_feed())
        )
        
        
        def start_climbing():
            config.climbing = True
            
        def stop_climbing():
            config.climbing = False
            
        def climbed():
            config.climbed = True
            
        def not_climbed():
            config.climbed = False
        
        Keymap.Climb.CLIMB_UP.and_(lambda: not config.climbing).onTrue(
            commands2.InstantCommand(lambda: start_climbing()).alongWith(
            command.EnableClimb(Robot.elevator, Robot.wrist, Robot.intake)
        )
        )
        
        # Keymap.Climb.CLIMB_UP.and_(lambda: config.climbing).onTrue(
        #     commands2.InstantCommand(lambda: stop_climbing()).alongWith(
        #     command.EnableClimb(Robot.elevator, Robot.wrist, Robot.intake)
        # )
        # )
        
        
        Keymap.Climb.CLIMB_DOWN.and_(lambda: config.climbing).onTrue(
            command.ClimbDown(Robot.elevator, Robot.wrist).alongWith(
                commands2.InstantCommand(lambda: climbed())
            )
        )
        
        Keymap.Climb.TRAP.and_(lambda: config.climbed).onTrue(command.ScoreTrap(Robot.elevator, Robot.wrist))
