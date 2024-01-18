import pytest
import config
import time
from subsystem import Elevator

class ElevatorMock(Elevator):

    length = 0.0
    voltage = 0.0

    def __init__(self):
        super().__init__()
        self.zeroed = False
        self.encoder = False
        self.motor_extend = False

    def init(self):
        self.zeroed = False
        self.encoder = True
        self.motor_extend = True

    def set_length(self, length: float):
        self.length = length

    def get_length(self):
        return self.length
    
    def set_voltage(self, voltage: float):
        self.voltage = voltage

    def get_voltage(self):
        return self.voltage
    
    def zero(self):
        self.zeroed = True
        self.length = 0.0


elevator = ElevatorMock()

# def test_elevator_init():
#     elevator.init()
#     time.sleep(1)
#     assert elevator.zeroed == False
#     assert elevator.encoder != False
#     assert elevator.motor_extend != False

def test_elevator_len():
    elevator.set_length(0.5)
    time.sleep(1)
    assert round(elevator.get_length(), 1) == 0.5

def test_elevator_zero():
    elevator.zero()
    time.sleep(1)
    assert elevator.zeroed == True
    print(elevator.get_length()) # REM IF PASSED
    assert round(elevator.get_length(), 1) == 0.0

def test_set_voltage():
    elevator.set_voltage(6)
    time.sleep(1)
    assert round(elevator.get_voltage()) == 6