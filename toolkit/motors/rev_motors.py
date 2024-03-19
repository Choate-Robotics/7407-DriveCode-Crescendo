from __future__ import annotations

import time  # noqa

import LocalLogger  # noqa
import rev
from rev import CANSparkMax, REVLibError, SparkMaxPIDController, SparkMaxRelativeEncoder
from wpilib import TimedRobot

import config
from toolkit.motor import PIDMotor
from units.SI import (  # noqa
    radians,
    radians_per_second,
    rotations,
    rotations_per_second,
    seconds,
)

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

    def __init__(
        self,
        k_P: float = None,
        k_I: float = None,
        k_D: float = None,
        k_F: float = None,
        output_range: tuple[float, float] = None,
        idle_mode: CANSparkMax.IdleMode = None,
    ):
        self.k_P = k_P
        self.k_I = k_I
        self.k_D = k_D
        self.k_F = k_F
        self.output_range = output_range
        self.idle_mode = idle_mode


class RevPeriodicFrames:
    def k0():
        """
        Applied Output, Faults, Sticky Faults, isFollower

        Default Period: 10ms
        """
        return CANSparkMax.PeriodicFrame.kStatus0

    def k1():
        """
        Motor Velocity, Motor Current, Motor Voltage, Motor Temperature

        Default Period: 20ms
        """
        return CANSparkMax.PeriodicFrame.kStatus1

    def k2():
        """
        Motor Position

        Default Period: 20ms
        """
        return CANSparkMax.PeriodicFrame.kStatus2

    def k3():
        """
        Analog Sensor Voltage, Analog Sensor Position, Analog Sensor Velocity

        Default Period: 50ms
        """
        return CANSparkMax.PeriodicFrame.kStatus3

    def k4():
        """
        Alternate Encoder Position, Alternate Encoder Velocity

        Default Period: 20ms
        """
        return CANSparkMax.PeriodicFrame.kStatus4

    def k5():
        """
        Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Angle

        Default Period: 200ms
        """
        return CANSparkMax.PeriodicFrame.kStatus5

    def k6():
        """
        Duty Cycle Absolute Encoder Velocity, Duty Cycle Absolute Encoder Frequency

        Default Period: 200ms
        """
        return CANSparkMax.PeriodicFrame.kStatus6


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
    _max_period_rev = 32767

    _optimized_basic_period_rev = 15

    def __init__(
        self,
        can_id: int,
        inverted: bool = False,
        brushless: bool = True,
        config: SparkMaxConfig = None,
        config_others: list[SparkMaxConfig] = None,
    ):
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

        self._logger = LocalLogger(f"SparkMax: {self._can_id}")

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
            CANSparkMax.MotorType.kBrushless
            if self._brushless or TimedRobot.isSimulation()
            else CANSparkMax.MotorType.kBrushed,
        )

        # Set pid controller
        self.pid_controller = self.motor.getPIDController() if self._brushless else None
        self.encoder = self.motor.getEncoder() if self._brushless else None

        # self.motor.restoreFactoryDefaults(True)

        time.sleep(0.5) if not TimedRobot.isSimulation() else None

        # Use the default config
        if self._configs[0] is not None and self._brushless:
            for enum, _config in enumerate(self._configs):
                time.sleep(0.5) if not TimedRobot.isSimulation() else None
                self._set_config(_config, enum)

        self.motor.setInverted(self._inverted)

        time.sleep(0.5) if not TimedRobot.isSimulation() else None
        self.motor.burnFlash()

        time.sleep(0.25) if not TimedRobot.isSimulation() else None

        self._has_init_run = True
        self._logger.complete("Initialized")

    def add_config_option(self, config: SparkMaxConfig):
        """
        Adds a config to self._configs
        :param config: Config to be added
        :return: None
        """

        self._configs.append(config)

    def set_average_depth(self, depth: int):
        result = self.encoder.setAverageDepth(depth)
        self.error_check(result)

    def set_measurement_period(self, period: int):
        result = self.encoder.setMeasurementPeriod(period)
        self.error_check(result)

    def set_motor_config(self, config_index: int = 0):
        """
        Set a config to SparkMax
        :param config_index: The config index

        """

        self._set_config(
            self._configs[config_index], config_index
        ) if self._brushless else None

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
                if error == REVLibError.kHALError:
                    print(f"SparkMax Error {self._can_id}: {error}")
                    return
                raise RuntimeError(f"SparkMax Error: {error}")

    def abs_encoder(self):
        if self._abs_encoder is None:
            self._abs_encoder = self.motor.getAbsoluteEncoder(
                rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
            )
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
            self._abs_encoder = self.motor.getAbsoluteEncoder(
                rev.SparkAbsoluteEncoder.Type.kDutyCycle
            )

        return self._abs_encoder

    def set_target_position(self, pos: rotations, arbff: float = 0, slot: int = 0):
        """
        Sets the target position of the motor controller in rotations

        Args:
            pos (float): The target position of the motor controller in rotations
        """
        result = self.pid_controller.setReference(
            pos, CANSparkMax.ControlType.kPosition, arbFeedforward=arbff, pidSlot=slot
        )
        self.error_check(result)

    def set_target_velocity(
        self, vel: rotations_per_second, arbff: float = 0
    ):  # Rotations per minute??
        """
        Sets the target velocity of the motor controller in rotations per second

        Args:
            vel (float): The target velocity of the motor controller in rotations per second
        """
        result = self.pid_controller.setReference(
            vel, CANSparkMax.ControlType.kVelocity, arbFeedforward=arbff
        )
        self.error_check(result)

    def set_target_voltage(self, voltage: float):
        """
        Sets the target voltage of the motor controller in volts

        Args:
            voltage (float): The target voltage of the motor controller in volts
        """
        result = self.pid_controller.setReference(
            voltage, CANSparkMax.ControlType.kVoltage
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

    def maximize_frame_period_rev(self, frame: RevPeriodicFrames):
        self.motor.setPeriodicFramePeriod(frame, self._max_period_rev)

    def optimize_normal_sparkmax(self):
        """Removes Analog Sensor, Absolute Encoder, and Alternate Encoder frames"""
        self.maximize_frame_period_rev(RevPeriodicFrames.k3())
        self.maximize_frame_period_rev(RevPeriodicFrames.k4())
        self.maximize_frame_period_rev(RevPeriodicFrames.k5())
        self.maximize_frame_period_rev(RevPeriodicFrames.k6())
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k1(), self._optimized_basic_period_rev
        )
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k2(), self._optimized_basic_period_rev
        )

    def optimize_sparkmax_analog_sensor(self, ms: int = 20):
        """Removes Alternate Encoder, and Duty Cycle Absolute Encoder frames

        Args:
            ms (int, optional): analog sensor frame period. Defaults to 20.
        """
        self.maximize_frame_period_rev(RevPeriodicFrames.k4())
        self.maximize_frame_period_rev(RevPeriodicFrames.k5())
        self.maximize_frame_period_rev(RevPeriodicFrames.k6())
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k1(), self._optimized_basic_period_rev
        )
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k2(), self._optimized_basic_period_rev
        )
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k3(), ms)

    def optimize_sparkmax_absolute_encoder(self, ms: int = 50):
        """Removes Analog Sensor, Alternate Encoder, and Absolute Encoder Frequency frames

        Args:
            ms (int, optional): absolute encoder frame period. Defaults to 50.
        """
        self.maximize_frame_period_rev(RevPeriodicFrames.k3())
        self.maximize_frame_period_rev(RevPeriodicFrames.k4())
        self.maximize_frame_period_rev(RevPeriodicFrames.k6())
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k1(), self._optimized_basic_period_rev
        )
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k2(), self._optimized_basic_period_rev
        )
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k5(), ms)

    def optimize_sparkmax_absolute_encoder_all(self, ms: int = 100):
        """Removes Analog Sensor and Alternate Encoder frames. Enables all Absolute Encoder frames

            This will probably never be used, but it's here just in case

        Args:
            ms (int, optional): absolute encoder frame period. Defaults to 100.
        """
        self.maximize_frame_period_rev(RevPeriodicFrames.k3())
        self.maximize_frame_period_rev(RevPeriodicFrames.k4())
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k1(), self._optimized_basic_period_rev
        )
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k2(), self._optimized_basic_period_rev
        )
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k5(), ms)
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k6(), ms)

    def optimize_sparkmax_no_position(self):
        """Removes Motor Position, Analog Sensor, Alternate Encoder, and Duty Cycle Absolute Encoder frames"""
        self.maximize_frame_period_rev(RevPeriodicFrames.k2())
        self.maximize_frame_period_rev(RevPeriodicFrames.k3())
        self.maximize_frame_period_rev(RevPeriodicFrames.k4())
        self.maximize_frame_period_rev(RevPeriodicFrames.k5())
        self.maximize_frame_period_rev(RevPeriodicFrames.k6())
        self.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
        self.motor.setPeriodicFramePeriod(
            RevPeriodicFrames.k1(), self._optimized_basic_period_rev
        )

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
            self.error_check(
                self.pid_controller.setOutputRange(
                    config.output_range[0], config.output_range[1], slot
                )
            )
        if config.idle_mode is not None:
            self.error_check(self.motor.setIdleMode(config.idle_mode))
