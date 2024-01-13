from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from phoenix6 import *
import math

from toolkit.motor import PIDMotor
from units.SI import radians_per_second, radians, seconds, rotations
import units

radians_per_second_squared = float


class TalonFX(PIDMotor):
    _motor: hardware.TalonFX
    
    _config: configs.TalonFXConfigurator

    _foc: bool

    _inverted: bool

    def __init__(self, can_id: int, foc: bool = True, inverted: bool = False):
        super().__init__()
        self._motor = hardware.TalonFX(can_id)
        self._inverted = inverted
        self._foc = foc
        self._can_id = can_id
        self._config = self._motor.configurator
        reversed_config = configs.MotorOutputConfigs()
        reversed_config.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE if self._inverted else signals.InvertedValue.CLOCKWISE_POSITIVE
        self._config.apply(reversed_config)
        motion_magic_config = configs.MotionMagicConfigs()
        motion_magic_config.motion
        

    def get_sensor_position(self) -> rotations:
        return self._motor.get_position()

    def set_target_position(self, pos: radians, arbFF: float = 0.0) -> StatusCode.OK:
        return self._motor.set_control(controls.MotionMagicExpoDutyCycle(pos, self._foc, arbFF))

    def set_sensor_position(self, pos: radians) -> StatusCode.OK:
        return self._motor.set_position(pos)

    def set_target_velocity(self, vel: radians_per_second, accel: radians_per_second_squared = 0) -> StatusCode.OK:
        return self._motor.set_control(controls.MotionMagicVelocityVoltage(vel, accel, self._foc))

    def set_raw_output(self, x: float) -> StatusCode.OK:
        self._motor.set_control(controls.DutyCycleOut(x, self._foc))

    def follow(self, master: TalonFX, inverted: bool = False) -> StatusCode.OK:
        return self._motor.set_control(controls.Follower(master._can_id, inverted))
    
    def get_sensor_velocity(self) -> radians_per_second:
        return self._motor.get_velocity()
    
    def get_motor_current(self) -> float:
        return self._motor.get_torque_current()
