from units.SI import meters, radians
from robot_systems import Field
from utils import POIPose
from wpimath.geometry import Translation2d, Translation3d, Pose2d
# from constants.FieldPos import MidLine
from constants import field_width, FieldPos, drivetrain_length_with_bumpers, drivetrain_length
import math
import constants

coord = (meters, meters, radians)
waypoints = (meters, meters)
path = (coord, waypoints, coord)

start = Field.POI.Coordinates.Waypoints.Auto.kSubwooferLeft

initial = (0.68, field_width/2 + 2.6, math.radians(-120))

initial_shot_location = POIPose(Pose2d(initial[0] + 2.5, initial[1], math.radians(-180))).withRotation(-180)
shot_location = Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) - 1.75, 0.4))

shoot_first_note = (
    start,
    [],
    POIPose(Pose2d(initial[0] + 0.2, initial[1], math.radians(-120)))
)

get_second_note = (
    initial,
    [Field.POI.Coordinates.Structures.Scoring.kAmp.withOffset(Translation2d(0, 0.6))],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(0, -0.2)).withRotation(-180)
)

shoot_second_note = (
    get_second_note[2],
    [
        # Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(
        # Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) + 0.1, 0.3))
        ],
    shot_location.withOffset(Translation2d(0.5, 0.35))
)

get_third_note = (
    shoot_second_note[2],
    [
        shoot_second_note[2].withOffset(Translation2d(1, 0))
        ],
    Field.POI.Coordinates.Notes.MidLine.kMidLeft.withOffset(Translation2d((-2 * constants.drivetrain_length / 3) + 0.5, 0)).withRotation(-135)
)

shoot_third_note = (
    get_third_note[2].withRotation(-135),
    [
        # Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(
        # Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) + 0.1, 0.7))
        ],
    shot_location
)

get_fourth_note = (
    shoot_second_note[2].withRotation(-180),
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft.withOffset(Translation2d(-0.4, 0)).withRotation(0)
)

get_fifth_note = (
    get_fourth_note[2].withRotation(-70),
    [Field.POI.Coordinates.Notes.Wing.kCenter.withOffset(Translation2d(-constants.drivetrain_length, -1*constants.FieldPos.Wing.note_gap/6))],
    Field.POI.Coordinates.Notes.Wing.kCenter.withOffset(Translation2d(0, -0.125)).withRotation(0)
)