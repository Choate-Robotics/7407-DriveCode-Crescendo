from builtins import type
from dataclasses import dataclass
from typing import Optional

from rev import CANSparkMax, SparkMaxPIDController, SparkMaxRelativeEncoder, SparkMaxAlternateEncoder

from toolkit.motor import PIDMotor
from units.SI import radians, radians_per_second, seconds, rotations_per_second, \
    rotations
    
rev = float
minute = float
rad = float
s = float

from units import Unum

# from toolkit.motors.ctre_motors import hundred_ms

hundred_ms = float


@dataclass
class SparkMaxConfig:
    """
    Configuration for a SparkMax motor controller

    Args:
        kP: Proportional gain
        kI: Integral gain
        kD: Derivative gain
        kF: Feedforward gain
        output_range: The minimum and maximum output of the controller as (min: float, max: float)
        idle_mode: Whether to brake or coast when the motor is not moving
    """

    k_P: Optional[float] = None
    k_I: Optional[float] = None
    k_D: Optional[float] = None
    k_F: Optional[float] = None
    output_range: Optional[tuple[float, float]] = None
    idle_mode: Optional[CANSparkMax.IdleMode] = None


# rev_sensor_unit = Unum.unit("rev_sensor_u", rev / 4096, "rev sensor unit")
# rev_sensor_vel_unit = rev_sensor_unit / hundred_ms
# rev_sensor_accel_unit = rev_sensor_vel_unit / s

# k_sensor_pos_to_radians = rev.asNumber(rad)
# k_radians_to_sensor_pos = radians.asNumber(rev)
# k_sensor_vel_to_rad_per_sec = (rev / minute).asNumber(rad / s)
# k_rad_per_sec_to_sensor_vel = (rad / s).asNumber(rev / minute)


class SparkMax(PIDMotor):
    """
    Wrapper class for the SparkMax motor controller
    """
    motor: CANSparkMax
    encoder: SparkMaxRelativeEncoder
    pid_controller: SparkMaxPIDController

    def __init__(self, can_id: int, inverted: bool = True, brushless: bool = True, config: SparkMaxConfig = None):
        """

        Args:
            can_id (int): The CAN ID of the motor controller
            inverted (bool, optional): Whether the motor is inverted. Defaults to True.
            brushless (bool, optional): Whether the motor is brushless. Defaults to True.
            config (SparkMaxConfig, None): The configuration for the motor controller. Defaults to None.
        """
        super().__init__()
        self._can_id = can_id
        self._inverted = inverted
        self._brushless = brushless
        self._config = config

    def init(self):
        """
        Initializes the motor controller, pid controller, and encoder
        """
        self.motor = CANSparkMax(
            self._can_id,
            CANSparkMax.MotorType.kBrushless if self._brushless else CANSparkMax.MotorType.kBrushed
        )
        self.motor.setInverted(self._inverted)
        self.pid_controller = self.motor.getPIDController()
        self.encoder = self.motor.getEncoder()
        self._set_config(self._config)

    def set_raw_output(self, x: float):
        """
        Sets the raw output of the motor controller

        Args:
            x (float): The output of the motor controller (between -1 and 1)
        """
        self.motor.set(x)

    def set_target_position(self, pos: rotations):
        """
        Sets the target position of the motor controller in rotations

        Args:
            pos (float): The target position of the motor controller in rotations
        """
        self.pid_controller.setReference(pos, CANSparkMax.ControlType.kPosition)

    def set_target_velocity(self, vel: rotations_per_second):  # Rotations per minute??
        """
        Sets the target velocity of the motor controller in rotations per second

        Args:
            vel (float): The target velocity of the motor controller in rotations per second
        """
        self.pid_controller.setReference(vel, CANSparkMax.ControlType.kVelocity)

    def get_sensor_position(self) -> rotations:
        """
        Gets the sensor position of the motor controller in rotations

        Returns:
            (rotations): The sensor position of the motor controller in rotations
        """
        return self.encoder.getPosition()

    def set_sensor_position(self, pos: rotations):
        """
        Sets the sensor position of the motor controller in rotations

        Args:
            pos (rotations): The sensor position of the motor controller in rotations
        """
        self.encoder.setPosition(pos)

    def get_sensor_velocity(self) -> rotations_per_second:
        """
        Gets the sensor velocity of the motor controller in rotations per second

        Returns:
            (rotations_per_second): The sensor velocity of the motor controller in rotations per second
        """
        return self.encoder.getVelocity()

    def _set_config(self, config: SparkMaxConfig):
        if config is None:
            return
        if config.k_P is not None:
            self.pid_controller.setP(config.k_P)
        if config.k_I is not None:
            self.pid_controller.setI(config.k_I)
        if config.k_D is not None:
            self.pid_controller.setD(config.k_D)
        if config.k_F is not None:
            self.pid_controller.setFF(config.k_F)
        if config.output_range is not None:
            self.pid_controller.setOutputRange(config.output_range[0], config.output_range[1])
        if config.idle_mode is not None:
            self.motor.setIdleMode(config.idle_mode)
