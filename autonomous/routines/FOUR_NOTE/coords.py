from units.SI import meters, radians, inches_to_meters
from robot_systems import Field
from wpimath.geometry import Translation2d
import constants
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (1.9 - constants.drivetrain_length_with_bumpers/2, constants.field_width/2, math.radians(-180))

get_first_note: path = (
    initial,
    [],
    Field.POI.Coordinates.Notes.Wing.kRight.withOffset(Translation2d(-2*constants.drivetrain_length/3, 0))
)

get_second_note: path = (
    get_first_note[2],
    [Field.POI.Coordinates.Notes.Wing.kRight.withOffset(Translation2d(-constants.drivetrain_length, -5*constants.FieldPos.Wing.note_gap/6))],
    Field.POI.Coordinates.Notes.Wing.kCenter.withOffset(Translation2d(-constants.drivetrain_length/2, 0))
)

get_third_note: path = (
    get_second_note[2],
    [Field.POI.Coordinates.Notes.Wing.kCenter.withOffset(Translation2d(-constants.drivetrain_length, -5*constants.FieldPos.Wing.note_gap/6))],
    Field.POI.Coordinates.Notes.Wing.kLeft.withOffset(Translation2d(-constants.drivetrain_length/3, 0))
)

go_to_midline = (
    get_third_note[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft
)