import math
from unittest.mock import MagicMock

import pytest
from pytest import MonkeyPatch
from wpimath.geometry import Pose2d, Rotation2d

import config
import constants
from sensors.trajectory_calc import TrajectoryCalculator, speaker_target
from units.SI import radians


@pytest.fixture
def trajectory_calc():
    odometry = MagicMock()
    elevator = MagicMock()
    trajectory_calc = TrajectoryCalculator(odometry, elevator, speaker_target)
    return trajectory_calc


@pytest.mark.parametrize(
    "odometry, expected_angle",
    [
        (
            Pose2d(2, 2, Rotation2d(0)),
            0.7973980580777741,
        ),
        (
            Pose2d(3, 2, Rotation2d(0)),
            0.7344352839640675,
        ),
        (
            Pose2d(2, 2, Rotation2d(0)),
            0.7973980580777741,
        ),
        (
            Pose2d(1, 4, Rotation2d(0)),
            0.8567503782414659,
        ),
        (
            Pose2d(0, 2, Rotation2d(0)),
            1.203641775098618,
        ),
    ],
)
def test_update_no_air(
    odometry, expected_angle, trajectory_calc, monkeypatch: MonkeyPatch
):
    monkeypatch.setattr(constants, "c", 0.47)
    monkeypatch.setattr(constants, "g", 9.81)
    monkeypatch.setattr(trajectory_calc.odometry, "getPose", lambda: odometry)
    monkeypatch.setattr(trajectory_calc.elevator, "get_length", lambda: 0.0)
    # print(f"target: {odometry}", sys.stdout)
    # print(constants.c)
    assert trajectory_calc.calculate_angle_no_air() == pytest.approx(
        expected_angle, abs=math.radians(1)
    )


@pytest.mark.parametrize(
    "angle, odometry, expected_answer",
    [
        (
            0.3,
            Pose2d(4, 2, Rotation2d(0)),
            0.7878720297645241,
        ),
        (
            0.56,
            Pose2d(3, 7, Rotation2d(0)),
            2.2025600711060442,
        ),
        (
            0.9,
            Pose2d(2, 2, Rotation2d(0)),
            2.364689623026117,
        ),
        (
            0.52,
            Pose2d(1, 4, Rotation2d(0)),
            0.8666219138213125,
        ),
        (
            1.0031,
            Pose2d(3, 9, Rotation2d(0)),
            4.819134130136504,
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
    ans = trajectory_calc.run_sim(angle)
    assert ans == pytest.approx(expected_answer, abs=0.01)


@pytest.mark.parametrize(
    "odometry, expected_answer",
    [
        (Pose2d(2, 3, Rotation2d(0)), 0.6759875790800276),
        (Pose2d(1, 6.8, Rotation2d(0)), 0.45088840191512236),
        (Pose2d(2, 5.1, Rotation2d(0)), 0.5183718576288799),
        (Pose2d(0.7, 5.54, Rotation2d(0)), 0.5366447015503899),
        (Pose2d(7, 4, Rotation2d(0)), 0.43793129522068697),
        (Pose2d(4, 2, Rotation2d(0)), 0.42853615436478704),
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
    trajectory_calc.use_air_resistance = True
    angle = trajectory_calc.get_wrist_angle()
    assert angle.radians() == pytest.approx(expected_answer, abs=math.radians(2))


@pytest.mark.parametrize(
    "odometry, expected_angle",
    [
        (Pose2d(2, 3, Rotation2d(0)), 3.3089451143081465),
        (Pose2d(1, 6.8, Rotation2d(0)), 4.475265058044473),
        (Pose2d(2, 5.1, Rotation2d(0)), 4.02533819476293),
        (Pose2d(6, 2.8, Rotation2d(0)), 3.164566111786272),
        (Pose2d(7, 4, Rotation2d(0)), 3.3304388005935888),
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
    assert trajectory_calc.get_base_angle().radians() == pytest.approx(
        expected_angle, abs=math.radians(1)
    )
