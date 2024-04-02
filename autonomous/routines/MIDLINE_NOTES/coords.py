from units.SI import meters, radians
from robot_systems import Field
from utils import POIPose
from wpimath.geometry import Translation2d, Translation3d
# from constants.FieldPos import MidLine
from constants import field_width, FieldPos, drivetrain_length_with_bumpers, drivetrain_length
import math

coord = (meters, meters, radians)
waypoints = (meters, meters)
path = (coord, waypoints, coord)

initial = (1.9 - drivetrain_length_with_bumpers/2, 2.92, math.radians(-180))

shoot_first_note = (
    initial,
    [],
    Field.POI.Coordinates.Waypoints.Auto.kMidlineAutoScoring.withOffset(Translation2d(-0.25, 0))
)

get_second_note = (
    shoot_first_note[2].withRotation(-155),
    [Field.POI.Coordinates.Structures.Obstacles.kStageRightPost.withOffset(Translation3d(0, 1.75, 0)),],
    Field.POI.Coordinates.Notes.MidLine.kFarRight.withOffset(Translation2d((-2 * drivetrain_length / 3) + 0.7, 0))
)

shoot_second_note = (
    get_second_note[2],
    [Field.POI.Coordinates.Structures.Obstacles.kStageRightPost.withOffset(Translation3d(0, 1.75, 0)),],
    Field.POI.Coordinates.Waypoints.Auto.kMidlineAutoScoring.withRotation(-155)
)

get_third_note = (
    shoot_second_note[2].withRotation(-135),
    [Field.POI.Coordinates.Structures.Obstacles.kStageRightPost.withOffset(Translation3d(0, 1.25, 0)),
    #  Field.POI.Coordinates.Notes.MidLine.kMidRight.withOffset(Translation2d(-1, 0))
     ],
    Field.POI.Coordinates.Notes.MidLine.kMidRight.withOffset(Translation2d(0, 0)).withRotation(90)
)

shoot_third_note = (
    get_third_note[2].withRotation(-90),
    [Field.POI.Coordinates.Structures.Stage.kCenter.withOffset(Translation2d(0, 0.2))],
    Field.POI.Coordinates.Structures.Stage.kLeft.withRotation(-135)
)

get_fourth_note = (
    shoot_third_note[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kCenter.withOffset(Translation2d((-2 * drivetrain_length / 3) + 0.7, 0))
)