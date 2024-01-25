import pytest
import rev
from pytest import MonkeyPatch
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig

import config
import constants
from subsystem import Drivetrain
from unittest.mock import MagicMock


@pytest.fixture()
def elevator() -> Drivetrain:
    # Create a drivetrain, but it has mock
    # classes for its dependencies
    my_drivetrain = Drivetrain()
    my_drivetrain.n_front_left: SparkMax = MagicMock()
    my_drivetrain.n_front_right: SparkMax = MagicMock()
    my_drivetrain.n_back_left: SparkMax = MagicMock()
    my_drivetrain.n_back_right: SparkMax = MagicMock()
    my_drivetrain.encoder = MagicMock()
    return my_drivetrain

def test_elevator_dunder_init(drivetrain: Drivetrain):
    drivetrain.init()

    assert drivetrain.n_front_left != False
    assert drivetrain.n_front_right != False
    assert drivetrain.n_back_left != False
    assert drivetrain.n_back_right != False

def test_x_mode(drivetrain: Drivetrain):
    drivetrain.x_mode()

def test_get_abs(drivetrain: Drivetrain):
    abs_val = drivetrain.get_abs()

    assert abs_val[0] == drivetrain.n_front_left.get_abs()
    assert abs_val[1] == drivetrain.n_front_right.get_abs()
    assert abs_val[2] == drivetrain.n_back_left.get_abs()
    assert abs_val[3] == drivetrain.n_back_right.get_abs()