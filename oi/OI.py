
from utils import LocalLogger

import commands2
import command
from robot_systems import Robot, Sensors
from oi.keymap import Keymap

log = LocalLogger("OI")

class OI:
    
    
    
    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")
        
        Keymap.Drivetrain.RESET_GYRO.onTrue(command.DrivetrainZero(Robot.drivetrain)).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
