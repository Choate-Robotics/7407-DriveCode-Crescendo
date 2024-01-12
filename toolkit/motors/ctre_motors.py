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
    
    _foc: bool
    
    _inverted: bool
    
    
    def __init__(self, can_id: int, foc: bool = True, inverted: bool = False):
        super().__init__()
        self._motor = hardware.TalonFX(can_id)
        self._inverted = inverted
        self._foc = foc
        self._can_id = can_id

        
    def get_sensor_position(self) -> rotations:
        return self._motor.get_position()
    
    def set_target_position(self, pos: radians, arbFF: float = 0.0) -> StatusCode.OK:
        return self._motor.set_control(controls.MotionMagicExpoDutyCycle(pos * self._inverted, self._foc, arbFF))
    
    def set_sensor_position(self, pos: radians) -> StatusCode.OK:
        return self._motor.set_position(pos * self._inverted)
    
    def set_target_velocity(self, vel: radians_per_second) -> StatusCode.OK:
        return super().set_target_velocity(vel)
    
    def set_raw_output(self, x: float) -> StatusCode.OK:
        self._motor.set_control(controls.DutyCycleOut(x * self._inverted, self._foc))
        
    def follow(self, master: TalonFX) -> StatusCode.OK:
        return self._motor
