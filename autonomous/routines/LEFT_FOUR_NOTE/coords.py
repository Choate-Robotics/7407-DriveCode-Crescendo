from units.SI import meters, radians, inches_to_meters
from robot_systems import Field
from wpimath.geometry import Translation2d, Pose2d
import constants
from utils import POIPose
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (1.9 - constants.drivetrain_length_with_bumpers/2, constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap * 2, math.radians(-180))

get_first_note: path = (
    initial,
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft.withOffset(Translation2d((-2 * constants.drivetrain_length / 3) + 0.2, 0))
)

get_second_note: path = (
    get_first_note[2],
    [Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) + 0.1, 0.3))],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d((-2 * constants.drivetrain_length / 3) + 0.7, 0.2))
)

go_to_wing_boundary_1: path = (
    get_second_note[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) - 1, 0.1)),
)