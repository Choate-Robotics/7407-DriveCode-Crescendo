# import commands2

import command
from oi.keymap import Keymap
from robot_systems import Robot
from utils import LocalLogger

log = LocalLogger("OI")


class OI:
    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")

        Keymap.Drivetrain.RESET_GYRO.onTrue(
            command.DrivetrainZero(Robot.drivetrain)
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
