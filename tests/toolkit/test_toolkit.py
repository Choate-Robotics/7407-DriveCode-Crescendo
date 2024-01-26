# # from unittest.mock import MagicMock
import math

import pytest
from pytest import approx

#
# # import config
# # import constants
from toolkit.utils.toolkit_math import bounded_angle_diff, rotate_vector

#


@pytest.mark.parametrize(
    "angle_1, angle_2, expected",
    [
        (0, 1, 1),
        (3.141592653589793, 1.5707963267948966, -1.5707963267948966),
        (4, 6.783185307179586, 2.7831853071795),
        (3, 3, 0),
        (10.283185307179586, 4, 0),
    ],
)
def test_bounded_angle_diff(angle_1, angle_2, expected):
    answer = bounded_angle_diff(angle_1, angle_2)
    assert answer == approx(expected)


@pytest.mark.parametrize(
    "x, y, theta, expected",
    [
        (1, 1, 0, (1, 1)),
        (1, 1, math.pi / 2, (-1, 1)),
        (1, 1, math.pi, (-1, -1)),
        (1, 1, 3 * math.pi / 2, (1, -1)),
        (1, 1, 2 * math.pi, (1, 1)),
        (1, 1, math.pi / 4, (0, math.sqrt(2))),
    ],
)
def test_rotate_vector(x, y, theta, expected):
    answer = rotate_vector(x, y, theta)
    assert answer == approx(expected)
