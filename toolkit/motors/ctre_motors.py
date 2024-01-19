from __future__ import annotations

from phoenix6 import hardware, configs, signals, controls, StatusCode, StatusSignal

from toolkit.motor import PIDMotor
from units.SI import rotations, rotations_per_second

radians_per_second_squared = float

rotations_per_second_squared = float


class TalonConfig:
    kP: float
    kI: float
    kD: float
    kF: float
    kA: float
    current_limit: float
    break_mode: bool
    output_range: tuple[float, float]

    def __init__(self, kP: float, kI: float, kD: float, kF: float, kA: float, current_limit: int = 80,
                 brake_mode: bool = True, output_range: tuple[float, float] = (-1, 1)):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kA = kA
        self.current_limit = current_limit
        self.brake_mode = brake_mode
        self.output_range = output_range

    def _apply_settings(self, motor: hardware.TalonFX, inverted: bool = False):
        print('applying settings to Talon')
        talon_config = configs.TalonFXConfiguration()

        # PID
        pid = talon_config.slot0
        pid.k_p = self.kP
        pid.k_i = self.kI
        pid.k_d = self.kD
        pid.k_s = self.kF
        pid.k_a = self.kA

        # current limits
        current_limits_config = talon_config.current_limits
        current_limits_config.supply_current_limit = self.current_limit
        current_limits_config.supply_current_limit_enable = True if self.current_limit > 0 else False
        current_limits_config.supply_time_threshold = 1

        # brake mode
        brake_mode_config = talon_config.motor_output
        brake_mode_config.neutral_mode = signals.NeutralModeValue.BRAKE if self.brake_mode else signals.NeutralModeValue.COAST
        brake_mode_config.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE if inverted else signals.InvertedValue.CLOCKWISE_POSITIVE

        # motion magic
        magic = talon_config.motion_magic
        magic.motion_magic_acceleration = 400
        magic.motion_magic_jerk = 4000

        res = motor.configurator.apply(talon_config)
        if res != StatusCode.OK:
            print('error! config not applying')
            raise RuntimeError
        else:
            print('talon configured')


class TalonFX(PIDMotor):
    _motor: hardware.TalonFX

    _config: configs.TalonFXConfigurator

    _talon_config: TalonConfig = None

    _motor_pos: StatusSignal

    _motor_vel: StatusSignal

    _motor_current: StatusSignal

    _mm_v_v: controls.MotionMagicVelocityVoltage

    _mm_p_v: controls.MotionMagicVoltage

    _d_o: controls.DutyCycleOut

    _foc: bool

    _inverted: bool

    def __init__(self, can_id: int, foc: bool = True, inverted: bool = False, config: TalonConfig = None):
        self._inverted = inverted
        self._foc = foc
        self._can_id = can_id
        self._talon_config = config

    def init(self):
        print('Initializing TalonFX', self._can_id)
        self._motor = hardware.TalonFX(self._can_id, 'rio')
        self._config = self._motor.configurator
        self._motor_pos = self._motor.get_position()
        self._motor_vel = self._motor.get_velocity()
        self._motor_current = self._motor.get_torque_current()
        if self._talon_config is not None:
            ...
            self._talon_config._apply_settings(self._motor, self._inverted)
        self.__setup_controls()

    def __setup_controls(self):
        self._mm_v_v = controls.MotionMagicVelocityVoltage(0)
        self._mm_p_v = controls.MotionMagicVoltage(0)
        self._d_o = controls.DutyCycleOut(0)

    def get_sensor_position(self) -> rotations:
        self._motor_pos.refresh()
        return self._motor_pos.value

    def set_target_position(self, pos: rotations, arbFF: float = 0.0) -> StatusCode.OK:
        return self._motor.set_control(self._mm_p_v.with_position(pos))

    def set_sensor_position(self, pos: rotations) -> StatusCode.OK:
        return self._motor.set_position(pos)

    def set_target_velocity(self, vel: rotations_per_second, accel: rotations_per_second_squared = 0) -> StatusCode.OK:
        # print('going', vel, 'rotations per second')
        return self._motor.set_control(self._mm_v_v.with_velocity(vel))
        # return self._motor.set_control(controls.VoltageOut(vel))
        # return self._motor.set_control(controls.VelocityDutyCycle(.5, enable_foc=False))

    def set_raw_output(self, x: float) -> StatusCode.OK:
        self._motor.set_control(self._d_o.with_output(x))

    def follow(self, master: TalonFX, inverted: bool = False) -> StatusCode.OK:
        return self._motor.set_control(controls.Follower(master._can_id, inverted))

    def get_sensor_velocity(self) -> rotations_per_second:
        self._motor_vel.refresh()
        return self._motor_vel.value

    def get_motor_current(self) -> float:
        self._motor_current.refresh()
        return self._motor_current.value
