from utils import LocalLogger

from commands2 import button
import config

import command

from robot_systems import Robot

log = LocalLogger("IT")


class IT:

    @staticmethod
    def init() -> None:
        log.info("Initializing IT...")

    @staticmethod
    def map_systems():
        log.info("Mapping systems...")
        pass


    button.Trigger(lambda: Robot.intake.get_back_current() > config.intake_roller_current_limit and not Robot.intake.intake_running)\
    .debounce(config.intake_sensor_debounce).onTrue(command.RunIntake(Robot.intake))

    button.Trigger(Robot.intake.detect_note()).debounce(config.intake_sensor_debounce).onTrue(command.IntakeIdle(Robot.intake))