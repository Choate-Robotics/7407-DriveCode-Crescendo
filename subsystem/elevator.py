import rev

from toolkit import Subsystem
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig

import config

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
        """
        Sets the length of the elevator in meters

        Args:
            length (meters): Length in meters
        """
        
        current_length = self.get_length()

        if length > current_length:
            ff = config.elevator_feed_forward
        elif length < current_length:
            ff = -config.elevator_feed_forward
        else: ff = 0
        
        self.motor_extend.pid_controller.setReference(length, rev.CANSparkMax.ControlType.kPosition, arbFeedforward=ff)

    def get_length(self) -> float:
        # Gets length and returns in meters
        return self.motor_extend.get_sensor_position() * config.elevator_gear_ratio
    
    def set_motor_position(self, position: float) -> None:
        if position > 1:
            position = 1
        elif position < 0:
            position = 0
        self.motor_extend.set_sensor_position(position * config.elevator_max_rotation)

    def zero(self) -> None:
        # Reset the encoder to zero

        encoder_pos: float = self.encoder.getPosition()

        if encoder_pos > 0.5:
            encoder_pos = -(1 - encoder_pos)

        self.motor_extend.set_sensor_position(encoder_pos * config.elevator_gear_ratio)
        self.motor_extend.set_target_position(0)

        self.set_motor_position(config.elevator_auto_position)
        self.zeroed = True

    def set_voltage(self, voltage: float) -> None:
        self.motor_extend.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)

    def get_voltage(self) -> float:
        return self.motor_extend.motor.getAppliedOutput()

    def stop(self) -> None:
        self.motor_extend.set_raw_output(0)
