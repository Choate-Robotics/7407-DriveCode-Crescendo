import math
from math import pi
from unittest.mock import MagicMock

import pytest

import config
import constants
from subsystem import Wrist
from toolkit.motors.rev_motors import SparkMax
from toolkit.utils.toolkit_math import bounded_angle_diff


@pytest.fixture()
def wrist() -> Wrist:
    # Create a wrist, but it has mock
    # classes for its dependencies

    my_wrist = Wrist()
    my_wrist.wrist_motor: SparkMax = MagicMock()
    my_wrist.feed_motor: SparkMax = MagicMock()
    my_wrist.wrist_abs_encoder = MagicMock()
    return my_wrist


@pytest.mark.parametrize(
    "test_input, disabled",
    [
        (0.5, False),
        (0.2, True),
        (0.1, False),
        (0, False),
    ],
)
@pytest.mark.skip('need to fix the test when everything gets updated')
def test_wrist_set_wrist_angle(test_input, disabled, wrist: Wrist):
    # wrist.init()
    wrist.rotation_disabled = disabled
    wrist.set_wrist_angle(test_input)
    if disabled:
        wrist.wrist_motor.set_target_position.assert_not_called()
    else:
        wrist.wrist_motor.set_target_position.assert_called_with(
            (wrist.limit_angle(test_input) / (2 * pi)) * constants.wrist_gear_ratio,
            config.wrist_flat_ff * math.cos(test_input),
        )


@pytest.mark.parametrize(
    "test_input",
    [
        (3),
        (0.2),
        (0.1),
        (0),
    ],
)
def test_get_wrist_angle(test_input, wrist: Wrist):
    # wrist.wrist_motor.get_sensor_position.return_value = test_input
    # assert wrist.get_wrist_angle() == (test_input / constants.wrist_gear_ratio) * pi * 2
    
    wrist.wrist_abs_encoder.getPosition.return_value = test_input
    assert wrist.get_wrist_angle() == wrist.abs_to_radians(test_input - config.wrist_zeroed_pos)


@pytest.mark.parametrize(
    "current_state, goal, threshold, expected",
    [
        (0.5, 0.1, math.radians(2), False),
        (0.2, 0.2 - math.radians(1), math.radians(5), True),
        (0, 0.1, math.radians(2), False),
        (1.2, 1.2 + math.radians(1), math.radians(2), True),
        (1.2, 1.2 + math.radians(2), math.radians(1), False),
    ],
)
@pytest.mark.skip('need to fix the test when everything gets updated')
def test_is_at_angle(current_state, goal, threshold, expected, wrist: Wrist):
    wrist.wrist_abs_encoder.return_value = (
        current_state * constants.wrist_gear_ratio / 2 / pi
    )
    print(bounded_angle_diff(current_state, goal))
    assert wrist.is_at_angle(goal, threshold) == expected


@pytest.mark.parametrize(
    "test_input",
    [
        (0),
        (0.2),
        (0.1),
        (0),
    ],
)
def test_zero_wrist(test_input, wrist: Wrist):
    wrist.wrist_abs_encoder.getPosition.return_value = test_input
    wrist.zero_wrist()
    wrist.wrist_motor.set_sensor_position.assert_called_with(
        test_input * constants.wrist_gear_ratio
    )
    assert wrist.wrist_zeroed


@pytest.mark.parametrize(
    "test_input",
    [True, False],
)
def test_feed_in(test_input, wrist: Wrist):
    wrist.feed_disabled = test_input
    wrist.feed_in()
    if test_input:
        wrist.feed_motor.set_target_voltage.assert_not_called()
    else:
        wrist.feed_motor.set_target_voltage.assert_called_with(config.feeder_voltage_feed)


@pytest.mark.parametrize(
    "test_input",
    [True, False],
)
def test_feed_out(test_input, wrist: Wrist):
    wrist.feed_disabled = test_input
    wrist.feed_out()
    if test_input:
        wrist.feed_motor.set_target_voltage.assert_not_called()
    else:
        wrist.feed_motor.set_target_voltage.assert_called_with(-config.feeder_voltage_trap)
