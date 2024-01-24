from utils import LocalLogger

from commands2 import button, InstantCommand

from robot_systems import Robot, Sensors

log = LocalLogger("IT")


class IT:

    @staticmethod
    def init() -> None:
        log.info("Initializing IT...")

    @staticmethod
    def map_systems():
        log.info("Mapping systems...")
        
        def stop_limelight_pos():
            Sensors.limelight.cam_pos_moving = True
            
        def start_limelight_pos():
            Sensors.limelight.cam_pos_moving = False
        
        button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1)\
            .onTrue(InstantCommand(stop_limelight_pos))\
            .onFalse(InstantCommand(start_limelight_pos))
