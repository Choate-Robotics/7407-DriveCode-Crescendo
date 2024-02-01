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

initial: coord = (1.9, 7.5, math.pi/2)

amp_1: path = ( 
    initial,
    [],
    Field.POI.Coordinates.Structures.Scoring.kAmp.withRotation(math.pi/2),
)

get_first_note: path = (
    amp_1[2],
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft.withRotation(math.pi),
)

amp_2: path = (
    get_first_note[2],
    [],
    Field.POI.Coordinates.Structures.Scoring.kAmp.withRotation(math.pi/2),
)

get_second_note: path = (
    amp_2[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withRotation(math.pi),
)

shoot_second_note: path = (
    get_second_note[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(-4, -1)),
)

