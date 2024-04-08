from units.SI import meters, radians
from robot_systems import Field
from utils import POIPose
from wpimath.geometry import Translation2d
# from constants.FieldPos import MidLine
from constants import field_width, FieldPos, drivetrain_length_with_bumpers
import math
from typing import Tuple
import constants

coord = Tuple[meters, meters, radians]
waypoints = Tuple[meters, meters]
path = Tuple[coord, waypoints, coord]

initial = ((1.9 - constants.drivetrain_length_with_bumpers/2) - 0.7, (constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap * 2) - 0.2, math.radians(-120))

leave = (
    (initial[0], initial[1], math.radians(180)),
    [],
    (initial[0] + 2, initial[1], math.radians(180)),
)

