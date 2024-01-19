import pytest
from pytest import MonkeyPatch
import config
import constants
from unittest.mock import MagicMock
from sensors.trajectory_calc import TrajectoryCalculator

@pytest.fixture
def trajectory_calc():
    field_od = MagicMock()
    # drivetrain = MagicMock()
    elevator = MagicMock()
    trajectory_calc = TrajectoryCalculator(field_od, elevator)
    trajectory_calc.odometry = MagicMock()
    # distance to speaker (m)
    trajectory_calc.odometry.getPose.return_value.translation.return_value.distance.return_value = 5
    # current height of elevator (m)
    trajectory_calc.elevator.get_height.return_value = 0.5
    return trajectory_calc

@pytest.mark.parametrize("distance_to_target, delta_z, expected_angle", [
    (5, 0.7, 0.25076),
    (5, 0.5, 0.21081),
    (3, 0.8, 0.32738),
    (3, 0.4, 0.198727)
])

def test_calculate_angle_no_air(trajectory_calc, distance_to_target, delta_z, expected_angle):
    trajectory_calc.delta_z = delta_z
    trajectory_calc.distance_to_target = distance_to_target
    assert trajectory_calc.calculate_angle_no_air(distance_to_target, delta_z) == pytest.approx(expected_angle, 0.0001)

