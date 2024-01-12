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
        self.encoder: rev.SparkAbsoluteEncoder = rev.SparkAbsoluteEncoder(self.motor_extend.motor) # TODO: Make sure params work
        self.motor_extend.motor.setClosedLoopRampRate(config.elevator_ramp_rate)

    def set_length(self, length: float) -> None:
        """
        Sets the length of the elevator in meters

        Args:
            length (meters): Length in meters
        """
        
        current_length = self.get_length()

        if length > .3 * config.elevator_max_rotation:
            ff = .65 * (1 if current_length < length else -1)
        elif length < .3 * config.elevator_max_rotation and length < current_length:
            ff = -.75
        else: ff = 0
        
        self.motor_extend.pid_controller.setReference(length, rev.CANSparkMax.ControlType.kPosition, arbFeedforward=ff)

    def get_length(self) -> float:
        # Gets length and returns in meters
        return self.motor_extend.get_sensor_position()
    

    def set_motor_position(self, position: float) -> None:
        if position > 1:
            position = 1
        elif position < 0:
            position = 0
        self.motor_extend.set_sensor_position(position * config.elevator_max_rotation)

    def zero(self) -> None:
        # Reset the encoder to zero
        self.set_motor_position(config.elevator_auto_position)
        self.encoder.reset() # TODO: Sebby plz review this is for the encoder too
        self.zeroed = True

    def set_voltage(self, voltage: float) -> None:
        self.motor_extend.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)

    def get_voltage(self) -> float:
        return self.motor_extend.motor.getAppliedOutput()

    def stop(self) -> None:
        self.motor_extend.set_raw_output(0)
