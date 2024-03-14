from __future__ import annotations
import config
from utils import LocalLogger

from rev import CANSparkMax, SparkMaxPIDController, SparkMaxRelativeEncoder, SparkMaxAlternateEncoder, REVLibError
import rev
from toolkit.motor import PIDMotor
from units.SI import radians, radians_per_second, seconds, rotations_per_second, \
    rotations
import time
from wpilib import TimedRobot

hundred_ms = float

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
    def __init__(self, k_P: float = None, k_I: float = None, k_D: float = None,
                 k_F: float = None, output_range: tuple[float, float] = None,
                 idle_mode: CANSparkMax.IdleMode = None):
        self.k_P = k_P
        self.k_I = k_I
        self.k_D = k_D
        self.k_F = k_F
        self.output_range = output_range
        self.idle_mode = idle_mode


class SparkMax(PIDMotor):
    """
    Wrapper class for the SparkMax motor controller
    """
    motor: CANSparkMax
    encoder: SparkMaxRelativeEncoder
    pid_controller: SparkMaxPIDController
    _configs: list[SparkMaxConfig] = []
    _has_init_run: bool = False
    _logger: LocalLogger
    _abs_encoder = None
    _get_analog = None
    _is_init: bool

    def __init__(self, can_id: int, inverted: bool = False, brushless: bool = True, config: SparkMaxConfig = None, config_others: list[SparkMaxConfig] = None):
        """

        Args:
            can_id (int): The CAN ID of the motor controller
            inverted (bool, optional): Whether the motor is inverted. Defaults to True.
            brushless (bool, optional): Whether the motor is brushless. Defaults to True.
            config (SparkMaxConfig, None): The default configuration for the motor controller. Defaults to None.
        """

        self._can_id = can_id
        self._inverted = inverted
        self._brushless = brushless
        
        self._configs = []

        self._configs.append(config)
        
        if config_others is not None:
            for config in config_others:
                if isinstance(config, SparkMaxConfig):
                    self._configs.append(config)

        self._logger = LocalLogger(f'SparkMax: {self._can_id}')

        self._has_init_run = False

        self._abs_encoder = None

        self._get_analog = None

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
            CANSparkMax.MotorType.kBrushless if self._brushless or TimedRobot.isSimulation() else CANSparkMax.MotorType.kBrushed
        )

        # Set pid controller
        self.pid_controller = self.motor.getPIDController() if self._brushless else None
        self.encoder = self.motor.getEncoder() if self._brushless else None

        # self.motor.restoreFactoryDefaults(True)

        time.sleep(0.5) if not TimedRobot.isSimulation() else None

        # Use the default config
        if self._configs[0] is not None and self._brushless:
            for enum, config in enumerate(self._configs):
                time.sleep(0.5) if not TimedRobot.isSimulation() else None
                self._set_config(config, enum)

        self.motor.setInverted(self._inverted)
        
        
        
        time.sleep(0.5) if not TimedRobot.isSimulation() else None
        self.motor.burnFlash()

        self._has_init_run = True
        self._logger.complete("Initialized")

    def add_config_option(self, config: SparkMaxConfig):
        """
        Adds a config to self._configs
        :param config: Config to be added
        :return: None
        """

        self._configs.append(config)

    def set_motor_config(self, config_index: int = 0):
        """
        Set a config to SparkMax
        :param config_index: The config index

        """

        self._set_config(self._configs[config_index], config_index) if self._brushless else None

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
                    self._logger.error(f'Uncommon Error {error}')
            if config.DEBUG_MODE:
                if error == REVLibError.kHALError:
                    print(f'SparkMax Error {self._can_id}: {error}')
                    return
                raise RuntimeError(f'SparkMax Error: {error}')

    def abs_encoder(self):
        if self._abs_encoder is None:
            self._abs_encoder = self.motor.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        return self._abs_encoder

    def get_analog(self):
        if self._get_analog is None:
            self._get_analog = self.motor.getAnalog()
        return self._get_analog

    def set_raw_output(self, x: float):
        """
        Sets the raw output of the motor controller

        Args:
            x (float): The output of the motor controller (between -1 and 1)
        """
        self.motor.set(x)

    def get_absolute_encoder(self):
        if self._abs_encoder is None:
            self._abs_encoder = self.motor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)

        return self._abs_encoder

    def set_target_position(self, pos: rotations, arbff: float = 0, slot: int = 0):
        """
        Sets the target position of the motor controller in rotations

        Args:
            pos (float): The target position of the motor controller in rotations
        """
        result = self.pid_controller.setReference(pos, CANSparkMax.ControlType.kPosition, arbFeedforward=arbff, pidSlot=slot)
        self.error_check(result)

    def set_target_velocity(self, vel: rotations_per_second):  # Rotations per minute??
        """
        Sets the target velocity of the motor controller in rotations per second

        Args:
            vel (float): The target velocity of the motor controller in rotations per second
        """
        result = self.pid_controller.setReference(vel, CANSparkMax.ControlType.kVelocity)
        self.error_check(result)

    def set_target_voltage(self, voltage: float):
        """
        Sets the target voltage of the motor controller in volts

        Args:
            voltage (float): The target voltage of the motor controller in volts
        """
        result = self.pid_controller.setReference(voltage, CANSparkMax.ControlType.kVoltage)
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


    def _set_config(self, config: SparkMaxConfig, slot: int = 0):
        if config is None:
            return
        if config.k_P is not None:
            self.error_check(self.pid_controller.setP(config.k_P, slot))
        if config.k_I is not None:
            self.error_check(self.pid_controller.setI(config.k_I, slot))
        if config.k_D is not None:
            self.error_check(self.pid_controller.setD(config.k_D, slot))
        if config.k_F is not None:
            self.error_check(self.pid_controller.setFF(config.k_F, slot))
        if config.output_range is not None:
            self.error_check(self.pid_controller.setOutputRange(config.output_range[0], config.output_range[1], slot))
        if config.idle_mode is not None:
            self.error_check(self.motor.setIdleMode(config.idle_mode))
