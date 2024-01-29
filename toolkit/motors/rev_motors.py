from __future__ import annotations

# from builtins import type
from dataclasses import dataclass
from typing import Optional

import rev
from rev import CANSparkMax, REVLibError, SparkMaxPIDController, SparkMaxRelativeEncoder
from wpilib import TimedRobot

import config
from toolkit.motor import PIDMotor
from units.SI import rotations, rotations_per_second
from utils import LocalLogger

hundred_ms = float


CANSparkMax


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


class SparkMax(PIDMotor):
    """
    Wrapper class for the SparkMax motor controller
    """

    motor: CANSparkMax
    encoder: SparkMaxRelativeEncoder
    pid_controller: SparkMaxPIDController
    _has_init_run: bool = False
    _logger: LocalLogger
    _abs_encoder = None

    _is_init: bool

    def __init__(
        self,
        can_id: int,
        inverted: bool = True,
        brushless: bool = True,
        config: SparkMaxConfig = None,
    ):
        """

        Args:
            can_id (int): The CAN ID of the motor controller
            inverted (bool, optional): Whether the motor is inverted. Defaults to True.
            brushless (bool, optional): Whether the motor is brushless. Defaults to True.
            config (SparkMaxConfig, None): The configuration for the motor controller. Defaults to None.
        """
        self._can_id = can_id
        self._inverted = inverted
        self._brushless = brushless
        self._config = config
        self._logger = LocalLogger(f"SparkMax: {self._can_id}")

        self._logger = LocalLogger(f"SparkMax: {self._can_id}")

        self._has_init_run = False

        self._abs_encoder = None

    def init(self):
        """
        Initializes the motor controller, pid controller, and encoder
        """

        if self._has_init_run:
            self._logger.warn("Already initialized")
            return

        # if TimedRobot.isSimulation():
        #     raise RuntimeError("SparkMax cannot be used in simulation")

        self._logger.setup("Initializing")

        self.motor = CANSparkMax(
            self._can_id,
            CANSparkMax.MotorType.kBrushless
            if self._brushless
            else CANSparkMax.MotorType.kBrushed,
        )
        self.motor.setInverted(self._inverted)
        self.pid_controller = self.motor.getPIDController()
        self.encoder = self.motor.getEncoder()
        self._set_config(self._config)

        self._has_init_run = True
        self._logger.complete("Initialized")

    def error_check(self, error: REVLibError):
        if TimedRobot.isSimulation():
            return
        if error != REVLibError.kOk:
            match error:
                case REVLibError.kInvalid:
                    self._logger.error("Invalid")
                case REVLibError.kCANDisconnected:
                    self._logger.error("CAN Disconnected")
                case REVLibError.kInvalidCANId:
                    self._logger.error("Invalid CAN ID")
                case REVLibError.kSetpointOutOfRange:
                    self._logger.error("Setpoint Out of Range")
                case REVLibError.kHALError:
                    self._logger.error("HAL Error")
                case REVLibError.kError:
                    self._logger.error("General Error")
                case REVLibError.kTimeout:
                    self._logger.error("Timeout")
                case _:
                    self._logger.error(f"Uncommon Error {error}")
            if config.DEBUG_MODE:
                raise RuntimeError(f"SparkMax Error: {error}")

    def abs_encoder(self):
        if self._abs_encoder is None:
            self._abs_encoder = self.motor.getAbsoluteEncoder(
                rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
            )
        return self._abs_encoder

    def set_raw_output(self, x: float):
        """
        Sets the raw output of the motor controller

        Args:
            x (float): The output of the motor controller (between -1 and 1)
        """
        self.motor.set(x)

    def get_abs(self):
        if self._abs_encoder is None:
            self._abs_encoder = self.motor.getAbsoluteEncoder(
                rev.SparkAbsoluteEncoder.Type.kDutyCycle
            )

        return self._abs_encoder

    def set_target_position(self, pos: rotations):
        """
        Sets the target position of the motor controller in rotations

        Args:
            pos (float): The target position of the motor controller in rotations
        """
        result = self.pid_controller.setReference(
            pos, CANSparkMax.ControlType.kPosition
        )
        self.error_check(result)

    def set_target_velocity(self, vel: rotations_per_second):  # Rotations per minute??
        """
        Sets the target velocity of the motor controller in rotations per second

        Args:
            vel (float): The target velocity of the motor controller in rotations per second
        """
        result = self.pid_controller.setReference(
            vel, CANSparkMax.ControlType.kVelocity
        )
        self.error_check(result)

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
        result = self.encoder.setPosition(pos)
        self.error_check(result)

    def get_sensor_velocity(self) -> rotations_per_second:
        """
        Gets the sensor velocity of the motor controller in rotations per second

        Returns:
            (rotations_per_second): The sensor velocity of the motor controller in rotations per second
        """
        return self.encoder.getVelocity()

    def follow(self, master: SparkMax, inverted: bool = False) -> None:
        result = self.motor.follow(master.motor, inverted)
        self.error_check(result)

    def _set_config(self, config: SparkMaxConfig):
        if config is None:
            return
        if config.k_P is not None:
            self.error_check(self.pid_controller.setP(config.k_P))
        if config.k_I is not None:
            self.error_check(self.pid_controller.setI(config.k_I))
        if config.k_D is not None:
            self.error_check(self.pid_controller.setD(config.k_D))
        if config.k_F is not None:
            self.error_check(self.pid_controller.setFF(config.k_F))
        if config.output_range is not None:
            self.error_check(
                self.pid_controller.setOutputRange(
                    config.output_range[0], config.output_range[1]
                )
            )
        if config.idle_mode is not None:
            self.error_check(self.motor.setIdleMode(config.idle_mode))
