import pytest
import rev
from pytest import MonkeyPatch

import config
import constants
from subsystem import Elevator
from unittest.mock import MagicMock


def test_elevator_dunder_init():
    elevator = Elevator()
    elevator.init()
    # elevator.encoder = MagicMock()
    # elevator.motor_extend = MagicMock()
    assert elevator.zeroed == False
    assert elevator.encoder != False
    assert elevator.motor_extend != False


def test_elevator_init():
    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    elevator.motor_extend.motor = MagicMock()
    elevator.encoder = MagicMock()
    elevator.init()

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
def test_set_length(test_input):
    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    elevator.motor_extend.motor = MagicMock()
    elevator.motor_extend.pid_controller = MagicMock()
    elevator.init()
    elevator.set_length(test_input)
    elevator.motor_extend.pid_controller.setReference.assert_called_with(
        (test_input * constants.elevator_gear_ratio)
        / constants.elevator_driver_gear_circumfrance,
        rev.CANSparkMax.ControlType.kPosition,
        arbFeedforward=config.elevator_feed_forward,
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
def test_get_length(test_input):
    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    elevator.motor_extend.motor = MagicMock()
    elevator.motor_extend.pid_controller = MagicMock()
    elevator.init()
    elevator.get_length()
    elevator.motor_extend.get_sensor_position.assert_called()
    elevator.motor_extend.get_sensor_position.return_value = test_input
    assert (
        elevator.get_length()
        == (test_input / constants.elevator_gear_ratio)
        * constants.elevator_driver_gear_circumfrance
    )


@pytest.mark.parametrize(
    "test_input,actual", [(0.2, 0.2), (3, 1), (1, 1), (0, 0), (-0.2, 0)]
)
def test_set_motor_position(test_input, actual):
    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    elevator.motor_extend.motor = MagicMock()
    elevator.motor_extend.pid_controller = MagicMock()
    elevator.init()
    elevator.set_motor_position(test_input)
    elevator.motor_extend.set_sensor_position.assert_called_with(
        actual * config.elevator_max_rotation
    )


def test_zero():
    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    elevator.motor_extend.motor = MagicMock()
    elevator.motor_extend.pid_controller = MagicMock()
    elevator.init()
    elevator.zero()
    elevator.motor_extend.set_sensor_position.assert_called_with(
        elevator.encoder.getPosition() * constants.elevator_gear_ratio
    )
    assert elevator.zeroed == True


def test_set_voltage():
    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    elevator.motor_extend.motor = MagicMock()
    elevator.motor_extend.pid_controller = MagicMock()
    elevator.init()
    elevator.set_voltage(1)
    elevator.motor_extend.pid_controller.setReference.assert_called_with(
        1, rev.CANSparkMax.ControlType.kVoltage
    )


def test_get_voltage():
    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    elevator.motor_extend.motor = MagicMock()
    elevator.motor_extend.motor.getAppliedOutput.return_value = 1
    elevator.init()
    value = elevator.get_voltage()
    elevator.motor_extend.motor.getAppliedOutput.assert_called()
    assert value == 1


def test_stop(monkeypatch: MonkeyPatch):
    def mock_get_length(self):
        return 1

    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    elevator.motor_extend.motor = MagicMock()
    elevator.motor_extend.pid_controller = MagicMock()
    monkeypatch.setattr(Elevator, "get_length", mock_get_length)
    elevator.init()
    elevator.stop()
    elevator.motor_extend.pid_controller.setReference.assert_called_with(
        (1 * constants.elevator_gear_ratio)
        / constants.elevator_driver_gear_circumfrance,
        rev.CANSparkMax.ControlType.kPosition,
        arbFeedforward=config.elevator_feed_forward
    )
