from units.SI import meters, radians, inches_to_meters
from robot_systems import Field
from wpimath.geometry import Translation2d, Translation3d, Pose2d
import constants
from utils import POIPose
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial = (1.9 - constants.drivetrain_length_with_bumpers/2, constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap * 2, math.radians(-180))

get_first_note = (
    initial,
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft.withOffset(Translation2d((-2 * constants.drivetrain_length / 3) + 0.2, 0)).withRotation(-180)
)

get_second_note = (
    get_first_note[2],
    [Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) + 0.1, 0.3))],
    # [Field.POI.Coordinates.Structures.Obstacles.kStageLeftPost.withOffset(Translation3d(0, -0.8, 0))],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d((-2 * constants.drivetrain_length / 3) + 0.5, 0))
)

go_to_wing_boundary_1 = (
    get_second_note[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) - 2, 0.75)),
)

get_third_note = (
    go_to_wing_boundary_1[2],
    [Field.POI.Coordinates.Structures.Obstacles.kStageLeftPost.withOffset(Translation3d(0, -1.3, 0))],
    Field.POI.Coordinates.Notes.MidLine.kMidLeft.withOffset(Translation2d((-2 * constants.drivetrain_length / 3) + 0.5, -0.25))
)

go_to_wing_boundary_2 = (
    get_third_note[2],
    [Field.POI.Coordinates.Structures.Obstacles.kStageLeftPost.withOffset(Translation3d(0, -1.3, 0))],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(-(constants.FieldPos.MidLine.mid_line - constants.FieldPos.wing_boundary) - 2, 0.75)),
)