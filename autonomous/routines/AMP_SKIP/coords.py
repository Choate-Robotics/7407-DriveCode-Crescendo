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

initial = (1.9 - constants.drivetrain_length_with_bumpers/2, (constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap * 1.5) + 0.15, math.radians(-180))

initial_shot_location = POIPose(Pose2d(initial[0] + 2.5, initial[1], math.radians(-180))).withRotation(-180)
shot_location = POIPose(Pose2d(initial[0] + 2.5, initial[1], math.radians(163.5))).withRotation(163.5)

shoot_first_note = (
    initial,
    [],
    initial_shot_location
)

get_second_note = (
    shoot_first_note[2],
    [Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(
        Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) + 0.1, 0.25))],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(
        Translation2d((-2 * constants.drivetrain_length / 3) + 0.5, -0.125))
)

shoot_second_note = (
    get_second_note[2],
    [Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(
        Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) + 0.1, 0.3))],
    shot_location
)

get_third_note = (
    shoot_second_note[2],
    [Field.POI.Coordinates.Structures.Obstacles.kStageLeftPost.withOffset(Translation3d(-0.125, -1.65, 0))],
    Field.POI.Coordinates.Notes.MidLine.kMidLeft.withOffset(Translation2d((-2 * constants.drivetrain_length / 3) + 0.5, -0.05)).withRotation(-135)
)

shoot_third_note = (
    get_third_note[2].withRotation(-135),
    [Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(
        Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) + 0.1, 0.7))],
    shot_location
)

far_to_mid = (
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withRotation(-120),
    [],
    Field.POI.Coordinates.Notes.MidLine.kMidLeft.withOffset(Translation2d(0, -0.25)).withRotation(130)
)

mid_to_far = (
    Field.POI.Coordinates.Notes.MidLine.kMidLeft.withRotation(0),
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withRotation(0)
)