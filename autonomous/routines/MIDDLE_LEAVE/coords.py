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

initial: coord = (1.219, 2.543, math.pi)

leave: path = ( 
    initial,
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft.withOffset(Translation2d(0, -.7)),
)

