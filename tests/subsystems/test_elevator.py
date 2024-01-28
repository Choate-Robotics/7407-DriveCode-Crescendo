import pytest
import rev
from pytest import MonkeyPatch
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig

import config
import constants
from subsystem import Elevator
from unittest.mock import MagicMock


@pytest.fixture()
def elevator() -> Elevator:
    # Create a drivetrain, but it has mock
    # classes for its dependencies
    my_elevator = Elevator()
    my_elevator.motor_extend: SparkMax = MagicMock()
    my_elevator.motor_extend.motor = MagicMock()
    my_elevator.motor_extend_follower: SparkMax = MagicMock()
    my_elevator.encoder = MagicMock()
    my_elevator.motor_extend.pid_controller = MagicMock()
    # my_elevator.init()
    return my_elevator


def test_elevator_dunder_init(elevator: Elevator):
    elevator.init()

    # elevator.encoder = MagicMock()
    # elevator.motor_extend = MagicMock()
    assert elevator.zeroed == False
    # assert elevator.encoder != False
    assert elevator.motor_extend != False
    elevator.motor_extend.init.assert_called()
    elevator.motor_extend.get_abs.assert_called()
    elevator.motor_extend.motor.setClosedLoopRampRate.assert_called()


@pytest.mark.parametrize(
    "test_input",
    [
        (0.5),
        (0.2),
        (0.1),
        (0),
    ],
)
def test_set_length(test_input, elevator: Elevator):
    # elevator.init()
    elevator.set_length(test_input)
    elevator.motor_extend.set_target_position.assert_called_with(
        (test_input * constants.elevator_gear_ratio)
        / constants.elevator_driver_gear_circumference
    )


@pytest.mark.parametrize(
    "test_input",
    [
        (1),
        (10),
        (1000),
        (0),
    ],
)
def test_get_length(test_input, elevator: Elevator):
    # elevator.get_length()
    # elevator.motor_extend.get_sensor_position.assert_called()
    elevator.motor_extend.get_sensor_position.return_value = test_input
    assert (
            elevator.get_length()
            == (test_input / constants.elevator_gear_ratio)
            * constants.elevator_driver_gear_circumference
    )


@pytest.mark.parametrize(
    "test_input,actual", [(0.2, 0.2), (3, 1), (1, 1), (0, 0), (-0.2, 0)]
)
def test_set_motor_position(test_input, actual, elevator: Elevator):
    elevator.set_motor_position(test_input)
    elevator.motor_extend.set_sensor_position.assert_called_with(
        actual * config.elevator_max_rotation
    )


@pytest.mark.parametrize(
    "test_input",
    [
        (1),
        (10),
        (1000),
        (0),
    ],
)
def test_zero(test_input, elevator: Elevator):
    elevator.encoder.getPosition.return_value = test_input
    elevator.zero()
    elevator.motor_extend.set_sensor_position.assert_called_with(
        test_input * constants.elevator_max_length)
    assert elevator.zeroed == True


@pytest.mark.parametrize(
    "test_input",
    [
        (0.5),
        (0.2),
        (0.1),
        (0),
    ],
)
def test_set_voltage(test_input, elevator: Elevator):
    elevator.set_voltage(test_input)
    elevator.motor_extend.pid_controller.setReference.assert_called_with(
        test_input, rev.CANSparkMax.ControlType.kVoltage
    )


def test_get_voltage(elevator: Elevator):
    elevator.motor_extend.motor.getAppliedOutput.return_value = 1
    value = elevator.get_voltage()
    elevator.motor_extend.motor.getAppliedOutput.assert_called()
    assert value == 1


@pytest.mark.parametrize(
    "test_input",
    [
        (0.5),
        (1.2),
        (0.1),
        (0),
    ],
)
def test_stop(test_input, elevator: Elevator, monkeypatch: MonkeyPatch):
    def mock_get_length(self):
        return test_input

    monkeypatch.setattr(Elevator, "get_length", mock_get_length)
    elevator.stop()
    elevator.motor_extend.set_target_position.assert_called_with(
        (test_input * constants.elevator_gear_ratio)
        / constants.elevator_driver_gear_circumference)
