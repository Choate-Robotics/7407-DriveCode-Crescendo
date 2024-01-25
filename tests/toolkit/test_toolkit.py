# from unittest.mock import MagicMock
import math

import pytest

# import config
# import constants
from toolkit.utils.toolkit_math import bounded_angle_diff

# from pytest import MonkeyPatch


@pytest.mark.parametrize(
    "angle_1, angle_2, expected",
    [
        (0, 1, 1),
        (math.pi, math.pi / 2, -math.pi / 2),
        (4, 0.5, math.pi - 3.5),
        (3, 3, 0),
        (4 + 2 * math.pi, 4, 0),
    ],
)
def test_bounded_angle_diff(angle_1, angle_2, expected):
    assert bounded_angle_diff(angle_1, angle_2) == expected
