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
            # 0.7973980580777741,
            0.6230314456071464,
        ),
        (
            Pose2d(3, 2, Rotation2d(0)),
            0.46827249925680225,
        ),
        (
            Pose2d(2, 2, Rotation2d(0)),
            0.6230314456071464,
        ),
        (
            Pose2d(8, 4, Rotation2d(0)),
            0.24278394992238336,
        ),
        (
            Pose2d(0, 2, Rotation2d(0)),
            1.150505593228981,
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
            1.0831721765290356,
        ),
        (
            0.56,
            Pose2d(3, 7, Rotation2d(0)),
            2.906118552719307,
        ),
        (
            0.9,
            Pose2d(2, 2, Rotation2d(0)),
            2.550534924229002,
        ),
        (
            0.52,
            Pose2d(1, 4, Rotation2d(0)),
            0.9240229993732696,
        ),
        (
            1.0031,
            Pose2d(3, 9, Rotation2d(0)),
            8.618001782075913,
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
        (Pose2d(1, 6.8, Rotation2d(0)), 0.3719536891913054),
        (Pose2d(2, 5.1, Rotation2d(0)), 0.46319500678277675),
        (Pose2d(0.7, 5.54, Rotation2d(0)), 0.4855080238313629),
        (Pose2d(4, 2, Rotation2d(0)), 0.3850788259149285),
        (Pose2d(8, 4, Rotation2d(0)), 0.27380521336822344),
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
