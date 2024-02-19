import math
from unittest.mock import MagicMock

import pytest
from pytest import MonkeyPatch
from wpimath.geometry import Pose2d, Rotation2d

import config
import constants
from sensors.trajectory_calc import TrajectoryCalculator
from units.SI import radians


@pytest.fixture
def trajectory_calc():
    odometry = MagicMock()
    elevator = MagicMock()
    trajectory_calc = TrajectoryCalculator(odometry, elevator)
    return trajectory_calc


@pytest.mark.parametrize(
    "odometry, expected_angle",
    [
        (
            Pose2d(2, 2, Rotation2d(0)),
            # 0.7973980580777741,
            0.5341948209165754,
        ),
        (
            Pose2d(3, 2, Rotation2d(0)),
            0.41722037134225975,
        ),
        (
            Pose2d(2, 2, Rotation2d(0)),
            0.5341948209165754,
        ),
        (
            Pose2d(8, 4, Rotation2d(0)),
            0.3222604314966177,
        ),
        (
            Pose2d(0, 2, Rotation2d(0)),
            1.0500218558151266,
        ),
    ],
)
def test_update_no_air(
    odometry, expected_angle, trajectory_calc, monkeypatch: MonkeyPatch
):
    monkeypatch.setattr(config, "v0_flywheel", 15)
    monkeypatch.setattr(constants, "g", 9.8)
    monkeypatch.setattr(trajectory_calc.odometry, "getPose", lambda: odometry)
    monkeypatch.setattr(trajectory_calc.elevator, "get_length", lambda: 0.0)
    # print(f"target: {odometry}")
    # print(f"deltaz: {trajectory_calc.delta_z}")
    # print(constants.c)
    trajectory_calc.init()
    trajectory_calc.calculate_distance()
    trajectory_calc.calculate_vertical_distance()
    assert trajectory_calc.calculate_angle_no_air(
        trajectory_calc.distance_to_target, trajectory_calc.delta_z
    ) == pytest.approx(expected_angle, abs=math.radians(1))


@pytest.mark.parametrize(
    "angle, odometry, expected_answer",
    [
        (
            0.3,
            Pose2d(4, 2, Rotation2d(0)),
            0.7824631017024342,
        ),
        (
            0.56,
            Pose2d(3, 7, Rotation2d(0)),
            2.204325659276666,
        ),
        (
            0.9,
            Pose2d(2, 2, Rotation2d(0)),
            2.366038986942942,
        ),
        (
            0.52,
            Pose2d(1, 4, Rotation2d(0)),
            0.8685995404543275,
        ),
        (
            1.0031,
            Pose2d(3, 9, Rotation2d(0)),
            4.8249551686713605,
        ),
    ],
)
def test_run_sim(
    trajectory_calc, angle: radians, odometry, expected_answer, monkeypatch: MonkeyPatch
):
    monkeypatch.setattr(config, "v0_flywheel", 15)
    monkeypatch.setattr(constants, "g", 9.8)
    monkeypatch.setattr(constants, "c", 0.47)
    monkeypatch.setattr(constants, "rho_air", 1.28)
    monkeypatch.setattr(constants, "a", 14 * 0.0254 * 2 * 0.0254)
    monkeypatch.setattr(constants, "m", 0.235301)
    monkeypatch.setattr(trajectory_calc.odometry, "getPose", lambda: odometry)
    monkeypatch.setattr(trajectory_calc.elevator, "get_length", lambda: 0.0)
    trajectory_calc.init()
    trajectory_calc.calculate_distance()
    ans = trajectory_calc.run_sim(angle)
    assert ans == pytest.approx(expected_answer, abs=0.01)


@pytest.mark.parametrize(
    "odometry, expected_answer",
    [
        (Pose2d(2, 3, Rotation2d(0)), 0.5548955786706005),
        (Pose2d(1, 6.8, Rotation2d(0)), 0.374759756027031),
        (Pose2d(2, 5.1, Rotation2d(0)), 0.4230906580762768),
        (Pose2d(0.7, 5.54, Rotation2d(0)), 0.43853530986790956),
        (Pose2d(4, 2, Rotation2d(0)), 0.38005200933865474),
        (Pose2d(8, 4, Rotation2d(0)), 0.42643661632888796),
    ],
)
def test_update_shooter(
    trajectory_calc, odometry, expected_answer, monkeypatch: MonkeyPatch
):
    monkeypatch.setattr(config, "v0_flywheel", 15)
    monkeypatch.setattr(constants, "g", 9.8)
    monkeypatch.setattr(constants, "c", 0.47)
    monkeypatch.setattr(constants, "rho_air", 1.28)
    monkeypatch.setattr(constants, "a", 14 * 0.0254 * 2 * 0.0254)
    monkeypatch.setattr(constants, "m", 0.235301)
    monkeypatch.setattr(trajectory_calc.odometry, "getPose", lambda: odometry)
    monkeypatch.setattr(trajectory_calc.elevator, "get_length", lambda: 0.0)
    # trajectory_calc.use_air_resistance = True
    trajectory_calc.init(True)
    trajectory_calc.update()
    angle = trajectory_calc.get_theta()
    assert angle == pytest.approx(expected_answer, abs=math.radians(2))


@pytest.mark.parametrize(
    "odometry, expected_answer",
    [
        (Pose2d(2, 3, Rotation2d(0)), 0.5548955786706005),
        (Pose2d(1, 6.8, Rotation2d(0)), 0.374759756027031),
        (Pose2d(2, 5.1, Rotation2d(0)), 0.4230906580762768),
        (Pose2d(0.7, 5.54, Rotation2d(0)), 0.43853530986790956),
        (Pose2d(4, 2, Rotation2d(0)), 0.38005200933865474),
        (Pose2d(8, 4, Rotation2d(0)), 0.42643661632888796),
    ],
)
def test_update_lookup_shooter(
    trajectory_calc, odometry, expected_answer, monkeypatch: MonkeyPatch
):
    monkeypatch.setattr(config, "v0_flywheel", 15)
    monkeypatch.setattr(constants, "g", 9.8)
    monkeypatch.setattr(constants, "c", 0.47)
    monkeypatch.setattr(constants, "rho_air", 1.28)
    monkeypatch.setattr(constants, "a", 14 * 0.0254 * 2 * 0.0254)
    monkeypatch.setattr(constants, "m", 0.235301)
    monkeypatch.setattr(trajectory_calc.odometry, "getPose", lambda: odometry)
    monkeypatch.setattr(trajectory_calc.elevator, "get_length", lambda: 0.0)
    # trajectory_calc.use_air_resistance = True
    trajectory_calc.init(True, True)
    trajectory_calc.update()
    angle = trajectory_calc.get_theta()
    assert angle == pytest.approx(expected_answer, abs=math.radians(2))


@pytest.mark.parametrize(
    "odometry, expected_angle",
    [
        (Pose2d(2, 3, Rotation2d(0)), -2.9742401928714397),
        (Pose2d(1, 6.8, Rotation2d(0)), -1.8079202491351138),
        (Pose2d(2, 5.1, Rotation2d(0)), -2.2578471124166546),
        (Pose2d(6, 2.8, Rotation2d(0)), -3.1186191953933142),
        (Pose2d(7, 4, Rotation2d(0)), -2.9527465065859975),
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
    trajectory_calc.update_base()

    assert trajectory_calc.get_bot_theta().radians() == pytest.approx(
        expected_angle, abs=math.radians(1)
    )
