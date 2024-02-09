import math
from math import pi

import config
import constants
from toolkit.motors.rev_motors import SparkMax
from toolkit.subsystem import Subsystem
from toolkit.utils.toolkit_math import bounded_angle_diff
from units.SI import radians


class Wrist(Subsystem):
    def __init__(self):
        super().__init__()
        self.wrist_motor = SparkMax(
            can_id=config.wrist_motor_id, inverted=True, config=config.WRIST_CONFIG
        )

        self.feed_motor = SparkMax(
            can_id=config.feed_motor_id, inverted=True, config=config.FEED_CONFIG
        )
        self.note_staged: bool = True
        self.wrist_zeroed: bool = False
        self.rotation_disabled: bool = False
        self.feed_disabled: bool = False

        self.disable_rotation: bool = False

    def init(self):
        self.wrist_motor.init()
        self.wrist_motor.motor.setClosedLoopRampRate(constants.wrist_time_to_max_vel)
        self.wrist_abs_encoder = self.wrist_motor.abs_encoder()
        self.feed_motor.init()

    # wrist methods
    def set_wrist_angle(self, angle: radians):
        """
        Sets the wrist angle to the given position
        :param pos: The position to set the wrist to(float)
        :return: None
        """
        if not self.disable_rotation:
            self.wrist_motor.set_target_position(
                (angle / (pi * 2)) * constants.wrist_gear_ratio
            )

    def get_wrist_angle(self):
        """
        Gets the wrist rotation in radians
        :return:
        """
        return (
            (self.wrist_motor.get_sensor_position() / constants.wrist_gear_ratio)
            * pi
            * 2
        )

    def is_at_angle(self, angle: radians, threshold=math.radians(2)):
        """
        Checks if the wrist is at the given angle
        :param angle: The angle to check for
        :param threshold: The threshold to check for
        :return: True if the wrist is at the given angle, False otherwise
        """
        return abs(bounded_angle_diff(self.get_wrist_angle(), angle)) < threshold

    def zero_wrist(self) -> None:  # taken from cyrus' code
        # Reset the encoder to zero
        self.wrist_motor.set_sensor_position(
            self.wrist_abs_encoder.getPosition() * constants.wrist_gear_ratio
        )
        self.zeroed = True
        
    # feed in methods
    def feed_in(self):
        if not self.feed_disabled:
            self.feed_motor.set_target_velocity(config.feeder_velocity)

    def feed_out(self):
        if not self.feed_disabled:
            self.feed_motor.set_target_velocity(-(config.feeder_velocity))
    
    def stop_feed(self):
        self.feed_motor.set_target_velocity(0)

    def feed_note(self):
        if not self.feed_disabled:
            self.feed_motor.set_target_velocity(config.feeder_pass_velocity)
