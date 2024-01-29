import rev

import config
import constants

from toolkit.subsystem import Subsystem
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig

# TODO: CHANGE WHEN ROBOT IS BUILT
ELEVATOR_CONFIG = SparkMaxConfig(
    0.055, 0.0, 0.01, config.elevator_feed_forward, (-.5, .75), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

class Elevator(Subsystem):
    
    elevator_moving: bool

    def __init__(self) -> None:
            super().__init__()
            # Absolute encoder
            self.motor_extend: SparkMax = SparkMax(
                config.elevator_can_id, config=ELEVATOR_CONFIG, inverted=False
            )
            self.motor_extend_follower: SparkMax = SparkMax(
                config.elevator_can_id_2, config=ELEVATOR_CONFIG, inverted=False
            )
            self.zeroed: bool = False
            self.elevator_moving: bool = False

    def init(self) -> None:
        self.motor_extend.init()
        self.motor_extend_follower.init()

        self.encoder = self.motor_extend.get_abs()
        self.motor_extend.motor.setClosedLoopRampRate(config.elevator_ramp_rate)

        self.motor_extend_follower.motor.follow(self.motor_extend.motor, invert=False)

    def set_length(self, length: float) -> None:
        # Sets length in meters
        self.motor_extend.set_target_position(
            (length * constants.elevator_gear_ratio) / constants.elevator_driver_gear_circumference)

    def get_length(self) -> float:
        # Gets length and returns in meters
        return (
                    self.motor_extend.get_sensor_position() / constants.elevator_gear_ratio) * constants.elevator_driver_gear_circumference

    def set_motor_position(self, position: float) -> None:
        if position > 1:
            position = 1
        elif position < 0:
            position = 0

        self.motor_extend.set_sensor_position(position * config.elevator_max_rotation)

    def zero(self) -> None:
        # Reset the encoder to zero
        self.motor_extend.set_sensor_position(self.encoder.getPosition() * constants.elevator_max_length)
        self.zeroed = True

    def set_voltage(self, voltage: float) -> None:
        self.motor_extend.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)

    def get_voltage(self) -> float:
        return self.motor_extend.motor.getAppliedOutput()

    def stop(self) -> None:
        # Set elevator to directly where it is
        self.set_length(self.get_length())
