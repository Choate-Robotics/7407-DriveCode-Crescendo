import math
import sys
from unittest.mock import MagicMock

import pytest
from pytest import MonkeyPatch
from wpimath.geometry import Pose2d, Rotation2d

# import config
# import constants
# import utils.POI
# from sensors.field_odometry import FieldOdometry
from sensors.trajectory_calc import (  # Target,; TargetCriteria,; TargetVariable,
    TrajectoryCalculator,
    speaker_target,
)


@pytest.fixture
def trajectory_calc():
    odometry = MagicMock()
    # drivetrain = MagicMock()
    elevator = MagicMock()
    trajectory_calc = TrajectoryCalculator(odometry, elevator, speaker_target)
    # trajectory_calc.odometry = MagicMock()

    # distance to speaker (m)
    # trajectory_calc.odometry.getPose.return_value.translation.return_value.distance.return_value = (
    #    5
    # )
    # current height of elevator (m)
    # trajectory_calc.elevator.get_height.return_value = 0.5
    return trajectory_calc


@pytest.mark.parametrize(
    "odometry, expected_angle",
    [
        (
            Pose2d(4, 2, Rotation2d(0)),
            0.39275870608907804,
        ),
        (
            Pose2d(3, 0, Rotation2d(0)),
            0.3729935526560053,
        ),
        (
            Pose2d(2, 2, Rotation2d(0)),
            0.44281124347644474,
        ),
        (
            Pose2d(1, 4, Rotation2d(0)),
            0.7405675431,
        ),
        (
            Pose2d(0, 2, Rotation2d(0)),
            0.470293281,
        ),
    ],
)
def test_update_no_air(
    odometry, expected_angle, trajectory_calc, monkeypatch: MonkeyPatch
):
    monkeypatch.setattr(trajectory_calc.odometry, "getPose", lambda: odometry)
    monkeypatch.setattr(trajectory_calc.elevator, "get_length", lambda: 0.0)
    # trajectory_calc.delta_z = delta_z
    # trajectory_calc.distance_to_target = distance_to_target
    print(f"target: {odometry}", sys.stdout)
    assert trajectory_calc.calculate_angle_no_air() == pytest.approx(
        expected_angle, abs=math.radians(1)
    )


# @pytest.mark.parametrize(
#     "angle, target, expected_answer",
#     [
#         (
#             0.3,
#             Target(
#                 Pose3d(Translation3d(0, 0, 1.3), Rotation3d(0, 0, 0)),
#                 15,
#                 TargetVariable.X,
#                 5.88,
#                 lambda x, y: y[0] > 5.88,
#             ),
#             0.9114040891170673,
#         ),
#         (
#             0.56,
#             Target(
#                 Pose3d(Translation3d(0, 0, 1.3), Rotation3d(0, 0, 0)),
#                 15,
#                 TargetVariable.X,
#                 6.53,
#                 lambda x, y: y[0] > 6.53,
#             ),
#             0.9114040891170673,
#         ),
#         (
#             0.9,
#             Target(
#                 Pose3d(Translation3d(0, 0.3, 1.3), Rotation3d(0, 0, 0)),
#                 15,
#                 TargetVariable.X,
#                 5.88,
#                 lambda x, y: y[0] > 5.88,
#             ),
#             0.9114040891170673,
#         ),
#         (
#             0.52,
#             Target(
#                 Pose3d(Translation3d(0, 0.3, 1.3), Rotation3d(0, 0, 0)),
#                 15,
#                 TargetVariable.X,
#                 5.88,
#                 lambda x, y: y[0] > 5.88,
#             ),
#             0.9114040891170673,
#         ),
#         (
#             1.0031,
#             Target(
#                 Pose3d(Translation3d(0, 0.3, 1.3), Rotation3d(0, 0, 0)),
#                 15,
#                 TargetVariable.X,
#                 5.88,
#                 lambda x, y: y[0] > 5.88,
#             ),
#             0.9114040891170673,
#         ),
#         (0.56, 6.53, 2.6401846942215146),
#         (0.9, 7.8, 5.730080943722868),
#         (0.52, 2.1, 1.0700338538709877),
#         (1.0031, 1.15, 1.699898),
#     ],
# )
# def test_run_sim(
#     trajectory_calc, angle, target, expected_answer, monkeypatch: MonkeyPatch
# ):
#     monkeypatch.setattr(config, "v0_flywheel", 15)
#     monkeypatch.setattr(constants, "g", 9.8)
#     monkeypatch.setattr(constants, "c", 0.47)
#     monkeypatch.setattr(constants, "rho_air", 1.28)
#     monkeypatch.setattr(constants, "a", 14 * 0.0254 * 2 * 0.0254)
#     monkeypatch.setattr(constants, "m", 0.235301)
#     monkeypatch.setattr(constants, "speaker_z", 1.7)
#     monkeypatch.setattr(constants, "shooter_height", 0.0)
#
#     # trajectory_calc.distance_to_target = x_distance
#     # trajectory_calc.print_constants()
#     ans = trajectory_calc.run_sim(angle)
#     assert ans == pytest.approx(expected_answer, 0.01)
#
#
# @pytest.mark.parametrize(
#     "x_distance, y_distance, expected_answer",
#     [
#         (1.15, 1.35, 0.8918247212271702),
#         (5.27, 1.63, 0.4323481479683407),
#         (6.11, 1.67, 0.4234831594121902),
#         (4.86, 1.71, 0.45967112158043777),
#     ],
# )
# def test_update_shooter(
#     trajectory_calc, x_distance, y_distance, expected_answer, monkeypatch: MonkeyPatch
# ):
#     def mock_get_length():
#         return 0
#
#     def mock_pose():
#         return Pose2d(x_distance, 0, 0)
#
#     trajectory_calc.speaker = Pose2d(0, 0, 0)
#     monkeypatch.setattr(config, "v0_flywheel", 15)
#     monkeypatch.setattr(constants, "shooter_height", 0.0)
#     monkeypatch.setattr(trajectory_calc.elevator, "get_length", mock_get_length)
#     monkeypatch.setattr(trajectory_calc.odometry, "getPose", mock_pose)
#     monkeypatch.setattr(trajectory_calc, "speaker", Translation2d(0, 0))
#     monkeypatch.setattr(constants, "g", 9.8)
#     monkeypatch.setattr(constants, "c", 0.47)
#     monkeypatch.setattr(constants, "rho_air", 1.28)
#     monkeypatch.setattr(constants, "a", 14 * 0.0254 * 2 * 0.0254)
#     monkeypatch.setattr(constants, "m", 0.235301)
#     trajectory_calc.speaker_z = y_distance
#     trajectory_calc.distance_to_target = x_distance
#     trajectory_calc.delta_z = y_distance
#     # print(trajectory_calc.delta_z)
#     angle = trajectory_calc.update_shooter()
#     assert angle == pytest.approx(expected_answer, math.radians(2))
#
#


@pytest.mark.parametrize(
    "odometry, expected_angle",
    [
        (Pose2d(2, 3, Rotation2d(0)), 2.17795669397),
        (Pose2d(1, 6.8, Rotation2d(0)), -2.12213750822),
        (Pose2d(2, 5.1, Rotation2d(0)), 2.89376365937),
        (Pose2d(6, 2.8, Rotation2d(0)), 2.69713889916),
        (Pose2d(7, 4, Rotation2d(0)), 2.91682031799),
    ],
)
def test_update_base(
    odometry,
    expected_angle,
    trajectory_calc,
    monkeypatch: MonkeyPatch,
):
    monkeypatch.setattr(trajectory_calc.odometry, "getPose", lambda: odometry)
    trajectory_calc.init()
    assert trajectory_calc.update_base().radians() == pytest.approx(
        expected_angle, abs=math.radians(1)
    )
