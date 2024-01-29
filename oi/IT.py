from utils import LocalLogger


from commands2 import button, ParallelDeadlineGroup, WaitCommand, ParallelRaceGroup, InstantCommand
import config

import command

from robot_systems import Robot, Sensors

log = LocalLogger("IT")


class IT:

    @staticmethod
    def init() -> None:
        log.info("Initializing IT...")

    @staticmethod
    def map_systems():
        log.info("Mapping systems...")
        

        button.Trigger(lambda: Robot.intake.get_back_current() > config.intake_roller_current_limit and not Robot.intake.intake_running)\
        .debounce(config.intake_sensor_debounce).onTrue(
            ParallelRaceGroup(
                WaitCommand(3), 
                command.RunIntake(Robot.intake)
            ).andThen(command.IntakeIdle(Robot.intake))
        )
        
        def stop_limelight_pos():
            Sensors.limelight.cam_pos_moving = True
            
        def start_limelight_pos():
            Sensors.limelight.cam_pos_moving = False
        
        button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1)\
            .onTrue(InstantCommand(stop_limelight_pos))\
            .onFalse(InstantCommand(start_limelight_pos))

