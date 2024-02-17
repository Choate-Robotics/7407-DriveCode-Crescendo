from unittest.mock import MagicMock

import pytest
import rev
from pytest import MonkeyPatch

import config
import constants
from subsystem import Flywheel
from toolkit.motors.rev_motors import SparkMax


@pytest.fixture()
def flywheel() -> Flywheel:
    # Create a drivetrain, but it has mock
    # classes for its dependencies
    my_flywheel = Flywheel()
    my_flywheel.motor_1: SparkMax = MagicMock()
    my_flywheel.motor_1.motor = MagicMock()
    my_flywheel.motor_2: SparkMax = MagicMock()
    my_flywheel.motor_2.motor = MagicMock()
    my_flywheel.motor_1.pid_controller = MagicMock()
    my_flywheel.motor_2.pid_controller = MagicMock()
    # my_flywheel.init()
    return my_flywheel


def test_flywheel_init(flywheel: Flywheel):
    flywheel.init()
    flywheel.motor_1.init.assert_called()
    flywheel.motor_2.init.assert_called()
    assert flywheel.initialized is True
    

@pytest.mark.parametrize(
    "test_input_1, test_input_2",
    [
        (30, 20),
        (20, 40),
        (10, 60),
        (0, 80),
    ],
)
def test_set_velocity(test_input_1, test_input_2, flywheel: Flywheel):
    flywheel.set_velocity(test_input_1, 1)
    flywheel.set_velocity(test_input_2, 2)
    assert flywheel.top_flywheel_state.nextR() == [test_input_1]
    assert flywheel.bottom_flywheel_state.nextR() == [test_input_2]
    flywheel.set_velocity(test_input_1)
    assert flywheel.top_flywheel_state.nextR() == [test_input_1]
    assert flywheel.bottom_flywheel_state.nextR() == [test_input_1]
    
@pytest.mark.parametrize(
    "test_input_1, test_input_2",
    [
        (75, 20),
        (20, 40),
        (10, 60),
        (0, 80),
    ],
)
def test_set_velocity_linear(test_input_1, test_input_2, flywheel: Flywheel):
    flywheel.set_velocity_linear(test_input_1, 1)
    flywheel.set_velocity_linear(test_input_2, 2)
    assert flywheel.top_flywheel_state.nextR() == [flywheel.linear_velocity_to_angular_velocity(test_input_1)]
    assert flywheel.bottom_flywheel_state.nextR() == [flywheel.linear_velocity_to_angular_velocity(test_input_2)]
    flywheel.set_velocity_linear(test_input_1)
    assert flywheel.top_flywheel_state.nextR() == [flywheel.linear_velocity_to_angular_velocity(test_input_1)]
    assert flywheel.bottom_flywheel_state.nextR() == [flywheel.linear_velocity_to_angular_velocity(test_input_1)]



@pytest.mark.parametrize(
    "test_input",
    [
        (30),
        (20),
        (10),
        (0),
    ],
)
def test_get_velocity(test_input, flywheel: Flywheel):
    flywheel.motor_1.get_sensor_velocity.return_value = test_input
    assert flywheel.get_velocity(1) == flywheel.rpm_to_angular_velocity(test_input)
    assert flywheel.get_velocity_linear(1) == flywheel.rpm_to_angular_velocity(test_input) * constants.flywheel_radius_outer
    
    flywheel.motor_2.get_sensor_velocity.return_value = test_input
    assert flywheel.get_velocity(2) == flywheel.rpm_to_angular_velocity(test_input)
    assert flywheel.get_velocity_linear(2) == flywheel.rpm_to_angular_velocity(test_input) * constants.flywheel_radius_outer
    
    assert flywheel.get_velocity() == (flywheel.rpm_to_angular_velocity(test_input), flywheel.rpm_to_angular_velocity(test_input))
    
@pytest.mark.parametrize(
    "test_input",
    [
        (20),
        (50),
        (3),
        (0),
    ],
)
def test_set_voltage(test_input, flywheel: Flywheel):
    flywheel.set_voltage(test_input, 1)
    flywheel.motor_1.pid_controller.setReference.assert_called_with(test_input, rev.CANSparkMax.ControlType.kVoltage)
    flywheel.set_voltage(test_input, 2)
    flywheel.motor_2.pid_controller.setReference.assert_called_with(test_input, rev.CANSparkMax.ControlType.kVoltage)
    flywheel.set_voltage(test_input)
    flywheel.motor_1.pid_controller.setReference.assert_called_with(test_input, rev.CANSparkMax.ControlType.kVoltage)
    flywheel.motor_2.pid_controller.setReference.assert_called_with(test_input, rev.CANSparkMax.ControlType.kVoltage)

@pytest.mark.parametrize(
    "test_input",
    [
        (30),
        (20),
        (10),
        (0),
    ],
)
def test_get_voltage(test_input, flywheel: Flywheel):
    flywheel.motor_1.motor.getAppliedOutput.return_value = test_input
    assert flywheel.get_voltage(1) == test_input
    flywheel.motor_2.motor.getAppliedOutput.return_value = test_input
    flywheel.motor_2.get_sensor_velocity.return_value = test_input
    assert flywheel.get_voltage(2) == test_input
    
    assert flywheel.get_voltage() == (test_input, test_input)
    

@pytest.mark.parametrize(
    'velocity, expected, tolerance, result',
    [
        (20, 20, 2, True),
        (20, 18, 0.5, False),
        (20, 10, 2, False),
        (20, 30, 2, False),
    ]    
)
def test_within_velocity(velocity, expected, tolerance, result, flywheel: Flywheel):
    flywheel.get_velocity = MagicMock()
    flywheel.get_velocity.return_value = velocity
    assert flywheel.within_velocity(expected, tolerance, 1) == result
    flywheel.motor_2.get_sensor_velocity.return_value = velocity
    assert flywheel.within_velocity(expected, tolerance, 2) == result
    
def test_periodic(flywheel: Flywheel):
    flywheel.get_velocity = MagicMock()
    flywheel.get_velocity.return_value = 20
    flywheel.get_current = MagicMock()
    flywheel.get_current.return_value = 10
    flywheel.periodic()
    top_f = flywheel.top_flywheel_state.U(0)
    bottom_f = flywheel.bottom_flywheel_state.U(0)
    flywheel.motor_1.pid_controller.setReference.assert_called_with(top_f, rev.CANSparkMax.ControlType.kVoltage)
    flywheel.motor_2.pid_controller.setReference.assert_called_with(bottom_f, rev.CANSparkMax.ControlType.kVoltage)