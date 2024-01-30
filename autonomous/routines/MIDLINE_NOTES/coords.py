from units.SI import meters, radians
from robot_systems import Field
from utils import POIPose
from wpimath.geometry import Translation2d
# from constants.FieldPos import MidLine
from constants import field_length

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (1.9, .78, 0)

get_first_ring: path = (
    initial,
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarRight.withRotation(0),
)

come_back_to_shoot_first_ring: path = (
    get_first_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarRight.withOffset(Translation2d(-2.3,1)),
)

get_second_ring: path = (
    come_back_to_shoot_first_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kMidRight.withRotation(0),
)

come_back_to_shoot_second_ring: path = (
    get_second_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarRight.withOffset(Translation2d(-2.3,1)),
)

get_third_ring: path = (
    come_back_to_shoot_second_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kCenter.withRotation(0),
)

come_back_to_shoot_third_ring: path = (
    get_third_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kCenter.withOffset(Translation2d(-4, 1.2)),
)