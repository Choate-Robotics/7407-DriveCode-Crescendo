import pytest
from pytest import MonkeyPatch

import config
import constants
from subsystem import Intake
from unittest.mock import MagicMock

def test_intake_dunder_init():
    intake = Intake()
    intake.init
    assert intake.note_in_intake == False

def test_intake_init():
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.init()

    intake.inner_motor.init.assert_called()
    intake.outer_motor_back.init.assert_called()
    intake.outer_motor_front.init.assert_called()

@pytest.mark.parametrize(
        "test_input",
        [
            (1),
            (2),
            (3),
            (4)
        ]
)

def test_set_inner_velocity(test_input):
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.inner_motor = MagicMock()
    intake.init()
    intake.set_inner_velocity(test_input)
    intake.inner_motor.set_target_velocity.assert_called_with(test_input * constants.intake_inner_gear_ratio)

@pytest.mark.parametrize(
        "test_input",
        [
            (1),
            (2),
            (3),
            (4)
        ]
)

def test_set_outer_velocity(test_input):
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.init()
    intake.set_outer_velocity(test_input)
    intake.outer_motor_back.set_target_velocity.assert_called_with(test_input * constants.intake_outer_gear_ratio)
    intake.outer_motor_front.set_target_velocity.assert_called_with(test_input * constants.intake_outer_gear_ratio)

def test_detect_note():
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.beam_break = MagicMock()
    intake.init()
    intake.detect_note()
    intake.beam_break.get.assert_called()

def test_roll_in():
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.init()
    intake.roll_in()
    intake.outer_motor_back.set_target_velocity.assert_called_with(config.intake_outer_speed * constants.intake_outer_gear_ratio)
    intake.outer_motor_front.set_target_velocity.assert_called_with(config.intake_outer_speed * constants.intake_outer_gear_ratio)
    intake.inner_motor.set_target_velocity.assert_called_with(config.intake_inner_speed * constants.intake_inner_gear_ratio)

def test_roll_out():
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.init()
    intake.roll_out()
    intake.outer_motor_back.set_target_velocity.assert_called_with(-config.intake_outer_speed * constants.intake_outer_gear_ratio)
    intake.outer_motor_front.set_target_velocity.assert_called_with(-config.intake_outer_speed * constants.intake_outer_gear_ratio)
    intake.inner_motor.set_target_velocity.assert_called_with(-config.intake_inner_speed * constants.intake_inner_gear_ratio)

def test_idle_in():
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.init()
    intake.rollers_idle_in()
    intake.outer_motor_back.set_target_velocity.assert_called_with(config.intake_outer_idle_speed * constants.intake_outer_gear_ratio)
    intake.outer_motor_front.set_target_velocity.assert_called_with(config.intake_outer_idle_speed * constants.intake_outer_gear_ratio)
    intake.inner_motor.set_target_velocity.assert_called_with(config.intake_inner_idle_speed * constants.intake_inner_gear_ratio)

def test_idle_out():
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.init()
    intake.rollers_idle_out()
    intake.outer_motor_back.set_target_velocity.assert_called_with(-config.intake_outer_idle_speed * constants.intake_outer_gear_ratio)
    intake.outer_motor_front.set_target_velocity.assert_called_with(-config.intake_outer_idle_speed * constants.intake_outer_gear_ratio)
    intake.inner_motor.set_target_velocity.assert_called_with(-config.intake_inner_idle_speed * constants.intake_inner_gear_ratio)

def test_front_current():
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.outer_motor_front.motor = MagicMock()
    intake.init()
    intake.get_front_current()
    intake.outer_motor_front.motor.getOutputCurrent.assert_called()

def test_back_current():
    intake = Intake()
    intake.inner_motor = MagicMock()
    intake.outer_motor_back = MagicMock()
    intake.outer_motor_front = MagicMock()
    intake.outer_motor_back.motor = MagicMock()
    intake.init()
    intake.get_back_current()
    intake.outer_motor_back.motor.getOutputCurrent.assert_called()
