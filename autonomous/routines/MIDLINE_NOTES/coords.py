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

start = Field.POI.Coordinates.Waypoints.Auto.kSubwooferRight.get()

initial = (0.67, field_width/2 + 0.3, math.radians(120))

shoot_first_note = (
    initial,
    [],
    Field.POI.Coordinates.Structures.Obstacles.kStageCenterPost.withOffset(Translation3d(-0.5, 0.9, 0))
)

get_second_note = (
    initial,
    [
        # Field.POI.Coordinates.Structures.Obstacles.kStageRightPost.withOffset(Translation3d(0, 1.3, 0)),
     ],
    Field.POI.Coordinates.Notes.MidLine.kFarRight.withOffset(Translation2d(0.1, 0.18))  # 0.25
)

shoot_second_note = (
    get_second_note[2],
    [Field.POI.Coordinates.Structures.Obstacles.kStageRightPost.withOffset(Translation3d(0, 1.3, 0))],
    Field.POI.Coordinates.Structures.Obstacles.kStageCenterPost.withOffset(Translation3d(-0.1, 0.85, 0)).withRotation(-120)
)

get_third_note = (
    shoot_second_note[2].withRotation(-135),
    [Field.POI.Coordinates.Structures.Obstacles.kStageRightPost.withOffset(Translation3d(0, 1.75, 0))],
    Field.POI.Coordinates.Notes.MidLine.kMidRight.withOffset(Translation2d(-0.25, 0)).withRotation(90)
)

come_back_with_third = (
    get_third_note[2].withRotation(90),
    [Field.POI.Coordinates.Structures.Obstacles.kStageRightPost.withOffset(Translation3d(0, 1.5, 0))],
    Field.POI.Coordinates.Structures.Obstacles.kStageCenterPost.withOffset(Translation3d(-0.1, 1.25, 0)).withRotation(-180)
)

shoot_third_note = (
    get_third_note[2].withRotation(-90),
    [Field.POI.Coordinates.Structures.Stage.kCenter.withOffset(Translation2d(0, 0.6))],
    # [Field.POI.Coordinates.Structures.Stage.kCenter],
    Field.POI.Coordinates.Structures.Stage.kLeft.withOffset(Translation2d(-0.2, -0.2)).withRotation(-135)
)

get_fourth_note = (
    shoot_third_note[2],
    [Field.POI.Coordinates.Structures.Stage.kCenter],
    Field.POI.Coordinates.Notes.MidLine.kCenter.withOffset(Translation2d(0.1, 0))
)

shoot_fourth_note = (
    get_fourth_note[2],
    [Field.POI.Coordinates.Structures.Stage.kCenter],
    Field.POI.Coordinates.Structures.Stage.kLeft.withOffset(Translation2d(-0.2, -0.2)).withRotation(-135)
)