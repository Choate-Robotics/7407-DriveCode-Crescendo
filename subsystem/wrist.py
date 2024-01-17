import config
import constants
from toolkit.subsystem import Subsystem
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig
from units import radians
import rev
from math import pi
import math

class Wrist(Subsystem):
    def __init__(self):
        super().__init__()
        self.wrist_motor = SparkMax(
            can_id=constants.wrist_motor_id, inverted=True,
            config=config.WRIST_CONFIG
        )
        
        self.feed_motor = SparkMax(
            can_id=constants.wrist_motor_id, inverted=True,
            config=config.WRIST_CONFIG
        )

    def init(self):
        self.wrist_motor.init()
        self.wrist_motor.motor.setClosedLoopRampRate(constants.time_to_max)
        self.wrist_abs_encoder = self.wrist_motor.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.feed_motor.init()
    
    def set_wrist_angle(self, angle: radians):
        """
        Sets the wrist angle to the given position
        :param pos: The position to set the wrist to(float)
        :return: None
        """
        if not self.disable_rotation:
            self.wrist_motor.set_target_position((angle / (pi * 2)) * constants.wrist_gear_ratio)
            # self.wrist_motor.set_sensor_position((pos / (pi * 2)) * constants.wrist_gear_ratio)


    def get_wrist_angle(self):
        """
        Gets the wrist rotation in radians
        :return:
        """
        return (self.wrist_motor.get_sensor_position() / constants.wrist_gear_ratio) * pi * 2

    def is_at_angle(self, angle: radians, threshold=math.radians(2)):
        """
        Checks if the wrist is at the given angle
        :param angle: The angle to check for
        :param threshold: The threshold to check for
        :return: True if the wrist is at the given angle, False otherwise
        """
        return abs(self.get_wrist_angle() - angle) < threshold

    #from cyrus' code
    def zero(self) -> None:
        # Reset the encoder to zero
        self.wrist_motor.set_sensor_position(self.wrist_abs_encoder.getPosition() * constants.wrist_gear_ratio)
        self.zeroed = True