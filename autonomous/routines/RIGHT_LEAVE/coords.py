from units.SI import meters, radians
from robot_systems import Field
from utils import POIPose
from wpimath.geometry import Translation2d
# from constants.FieldPos import MidLine
from constants import field_width, FieldPos, drivetrain_length_with_bumpers
import math
from typing import Tuple

coord = Tuple[meters, meters, radians]
waypoints = Tuple[meters, meters]
path = Tuple[coord, waypoints, coord]

initial: coord = (0.484, 3.682, -2.101)

leave: path = ( 
    initial,
    [],
    (initial[0]+1.7, initial[1], math.pi),
)

