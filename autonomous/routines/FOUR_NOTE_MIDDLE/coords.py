from units.SI import meters, radians, inches_to_meters
from robot_systems import Field
from wpimath.geometry import Translation2d
import constants
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial = (1.9 - constants.drivetrain_length_with_bumpers/2, constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap, math.radians(-180))

get_first_note = (
    (initial[0], initial[1], math.radians(135)),
    [],
    Field.POI.Coordinates.Notes.Wing.kRight.withOffset(Translation2d(-constants.drivetrain_length/2, -0.12)).withRotation(-135)
)

get_second_note = (
    Field.POI.Coordinates.Notes.Wing.kRight.withOffset(Translation2d(-constants.drivetrain_length/2, 0)).withRotation(-90),
    [],
    Field.POI.Coordinates.Notes.Wing.kCenter.withOffset(Translation2d(-2*constants.drivetrain_length/3, 0)).withRotation(-90)
)

get_third_note = (
    get_second_note[2],
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft.withOffset(Translation2d(-2*constants.drivetrain_length/3, 0)).withRotation(-90)
)

go_to_midline = (
    Field.POI.Coordinates.Notes.Wing.kLeft.withOffset(Translation2d(-2*constants.drivetrain_length/3, 0)),
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d((-2 * constants.drivetrain_length / 3) - 1.25, 0))
)