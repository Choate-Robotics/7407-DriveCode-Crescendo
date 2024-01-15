from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from phoenix6 import hardware, configs, signals, controls, StatusCode, StatusSignal
import math

from toolkit.motor import PIDMotor
from units.SI import radians_per_second, radians, seconds, rotations, radians_to_rotations, rotations_per_second
import units

radians_per_second_squared = float

rotations_per_second_squared = float

class TalonConfig:
    
    kP: float
    kI: float
    kD: float
    kF: float
    current_limit: float
    break_mode: bool
    output_range: tuple[float, float]

    def __init__(self, kP: float, kI: float, kD: float, kF: float, current_limit: int = 0, brake_mode: bool = True, output_range: tuple[float, float] = (-1, 1)):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.current_limit = current_limit
        self.brake_mode = brake_mode
        self.output_range = output_range
    def _apply_settings(self, motor: hardware.TalonFX):
        print('applying settings to Talon')
        pid = configs.Slot0Configs()
        pid.k_p = self.kP
        pid.k_i = self.kI
        pid.k_d = self.kD
        pid.k_s = self.kF
        motor.configurator.apply(pid)
        current_limits_config = configs.CurrentLimitsConfigs()
        current_limits_config.supply_current_limit = self.current_limit
        current_limits_config.supply_current_limit_enable = True if self.current_limit > 0 else False
        current_limits_config.supply_time_threshold = 1
        motor.configurator.apply(current_limits_config)
        break_mode_config = configs.MotorOutputConfigs()
        break_mode_config.neutral_mode = signals.NeutralModeValue.BRAKE if self.brake_mode else signals.NeutralModeValue.COAST
        motor.configurator.apply(break_mode_config)


class TalonFX(PIDMotor):
    _motor: hardware.TalonFX
    
    _config: configs.TalonFXConfigurator
    
    _talon_config: TalonConfig = None
    
    _motor_pos: StatusSignal
    
    _motor_vel: StatusSignal
    
    _motor_current: StatusSignal

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
        reversed_config = configs.MotorOutputConfigs()
        reversed_config.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE if self._inverted else signals.InvertedValue.CLOCKWISE_POSITIVE
        self._config.apply(reversed_config)
        if self._talon_config is not None:
            self._talon_config._apply_settings(self._motor)

    def get_sensor_position(self) -> rotations:
        self._motor_pos.refresh()
        return self._motor_pos.value

    def set_target_position(self, pos: rotations, arbFF: float = 0.0) -> StatusCode.OK:
        return self._motor.set_control(controls.MotionMagicExpoDutyCycle(pos, self._foc, arbFF))

    def set_sensor_position(self, pos: rotations) -> StatusCode.OK:
        return self._motor.set_position(pos)

    def set_target_velocity(self, vel: rotations_per_second, accel: rotations_per_second_squared = 0) -> StatusCode.OK:
        
        return self._motor.set_control(controls.VelocityVoltage(vel, accel, self._foc))
        
    def set_raw_output(self, x: float) -> StatusCode.OK:
        self._motor.set_control(controls.DutyCycleOut(x, self._foc))

    def follow(self, master: TalonFX, inverted: bool = False) -> StatusCode.OK:
        return self._motor.set_control(controls.Follower(master._can_id, inverted))
    
    def get_sensor_velocity(self) -> rotations_per_second:
        self._motor_vel.refresh()
        return self._motor_vel.value
    
    def get_motor_current(self) -> float:
        self._motor_current.refresh()
        return self._motor_current.value
