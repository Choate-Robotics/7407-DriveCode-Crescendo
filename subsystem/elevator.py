import rev

import config
import constants

from toolkit import Subsystem
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig

# TODO: CHANGE WHEN ROBOT IS BUILT
ELEVATOR_CONFIG = SparkMaxConfig(
    0.055, 0.0, 0.01, 0.000, (-.5, .75), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

class Elevator(Subsystem):

    motor_extend: SparkMax = SparkMax(
        config.elevator_can_id, config=ELEVATOR_CONFIG, inverted=False
    )

    def __init__(self) -> None:
        super().__init__()
        # Absolute encoder
        self.zeroed: bool = False

    def init(self) -> None:
        self.motor_extend.init()
        self.encoder = self.motor_extend.get_abs()
        self.motor_extend.motor.setClosedLoopRampRate(config.elevator_ramp_rate)

    def set_length(self, length: float) -> None:
        # Sets length in meters
        self.motor_extend.pid_controller.setReference((length * constants.elevator_gear_ratio) / constants.elevator_driver_gear_circumfrance, rev.CANSparkMax.ControlType.kPosition, arbFeedforward=config.elevator_feed_forward)

    def get_length(self) -> float:
        # Gets length and returns in meters
        return (self.motor_extend.get_sensor_position() / constants.elevator_gear_ratio) * constants.elevator_driver_gear_circumfrance
    
    def set_motor_position(self, position: float) -> None:
        if position > 1:
            position = 1
        elif position < 0:
            position = 0

        self.motor_extend.set_sensor_position(position * config.elevator_max_rotation)

    def zero(self) -> None:
        # Reset the encoder to zero
        self.motor_extend.set_sensor_position(self.encoder.getPosition() * constants.elevator_gear_ratio)
        self.zeroed = True

    def set_voltage(self, voltage: float) -> None:
        self.motor_extend.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)

    def get_voltage(self) -> float:
        return self.motor_extend.motor.getAppliedOutput()

    def stop(self) -> None:
        # Set elevator to directly where it is
        self.set_length(self.get_length() * constants.elevator_gear_ratio)
        # Stop all output to remove any motor movement
        self.motor_extend.set_raw_output(0)