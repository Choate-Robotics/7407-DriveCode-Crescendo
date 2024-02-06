from units.SI import meters, radians
from robot_systems import Field
from utils import POIPose
from wpimath.geometry import Translation2d
# from constants.FieldPos import MidLine
from constants import field_length, FieldPos
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (1.9, Field.POI.Coordinates.Notes.Wing.kRight.get().Y(), math.pi)

get_first_ring: path = ( 
    initial,
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft,
)

come_back_to_shoot_first_ring: path = (
    get_first_ring[2],
    [],
    # Field.POI.Coordinates.Notes.MidLine.kFarRight.withOffset(Translation2d(2.3,1)),
    Field.POI.Coordinates.Notes.MidLine.kFarRight,
)

get_second_ring: path = (
    come_back_to_shoot_first_ring[2],
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft,
)

come_back_to_shoot_second_ring: path = (
    get_second_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarRight.withOffset(Translation2d(-2.3,-1)),
)

get_third_ring: path = (
    come_back_to_shoot_second_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kCenter,
)

come_back_to_shoot_third_ring: path = (
    get_third_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kCenter.withOffset(Translation2d(-4.5, 1.2)),
)