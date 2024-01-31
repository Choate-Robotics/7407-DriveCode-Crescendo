from units.SI import meters, radians, inches_to_meters
from robot_systems import Field
from wpimath.geometry import Translation2d
import constants
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (0, constants.field_width/2, math.radians(180))

get_first_note: path = (
    initial,
    [],
    Field.POI.Coordinates.Notes.Wing.kRight
)

get_second_note: path = (
    get_first_note[2],
    [],
    Field.POI.Coordinates.Notes.Wing.kCenter
)

get_third_note: path = (
    get_second_note[2],
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft
)