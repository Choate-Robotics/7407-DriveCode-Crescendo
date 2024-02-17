from utils import LocalLogger

import commands2
import command
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

        Keymap.Drivetrain.RESET_GYRO.onTrue(command.DrivetrainZero(Robot.drivetrain)).onFalse(
            command.DriveSwerveCustom(Robot.drivetrain))
        
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
