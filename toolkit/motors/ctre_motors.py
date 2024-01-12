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
    
    _active_control_mode: controls
    
    def __init__(self, can_id: int, foc: bool = True, inverted: bool = False):
        super().__init__()
        self._motor = hardware.TalonFX(can_id)
        self._inverted = inverted
        self._foc = foc
        self._can_id = can_id
        
    def __set_control_mode(self, mode: controls.DutyCycleOut):
        self._active_control_mode = mode
        
    def get_sensor_position(self) -> rotations:
        return self._motor.get_position()
    
    def set_target_position(self, pos: radians) -> StatusCode.OK:
        return self._motor.set_control()
    
    def set_target_velocity(self, vel: radians_per_second) -> StatusCode.OK:
        return super().set_target_velocity(vel)
    
    def set_raw_output(self, x: float) -> StatusCode.OK:
        self._motor.set_control(controls.DutyCycleOut(x * self._inverted, self._foc))
