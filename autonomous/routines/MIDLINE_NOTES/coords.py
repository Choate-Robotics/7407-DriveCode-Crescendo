from units.SI import meters, radians
from robot_systems import Field
from utils import POIPose
from wpimath.geometry import Translation2d
# from constants.FieldPos import MidLine
from constants import field_width, FieldPos, drivetrain_length_with_bumpers
import math

coord = (meters, meters, radians)
waypoints = (meters, meters)
path = (coord, waypoints, coord)

initial: coord = (1.9 - drivetrain_length_with_bumpers/2, field_width-5.373, math.pi)

come_out_shoot_preload: path = (
    initial,
    [],
    (initial[0]+2, initial[1], initial[2]),
)

get_first_ring: path = ( 
    come_out_shoot_preload[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarRight,
)

come_back_to_shoot_first_ring: path = (
    get_first_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarRight.withOffset(Translation2d(-4.3, -.7)),
)

get_second_ring: path = (
    come_back_to_shoot_first_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kMidRight,
)

come_back_to_shoot_second_ring: path = (
    get_second_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarRight.withOffset(Translation2d(-4.3, -.7)),
)

# get_third_ring: path = (
#     come_back_to_shoot_second_ring[2],
#     [],
#     Field.POI.Coordinates.Notes.MidLine.kCenter,
# )

# come_back_to_shoot_third_ring: path = (
#     get_third_ring[2],
#     [(FieldPos.Stage.stage_x, FieldPos.Stage.stage_y)],
#     Field.POI.Coordinates.Notes.MidLine.kCenter.withOffset(Translation2d(-4.5, -1.2)),
# )