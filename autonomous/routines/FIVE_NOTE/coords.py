from units.SI import meters, radians
from robot_systems import Field
import constants, math
from utils import POIPose
from wpimath.geometry import Translation2d

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (1.9, 5.5, 0)

get_second_note: path = (
    initial,
    [],
    Field.POI.Coordinates.Notes.Wing.kCenter.withRotation(0),
)

get_third_note: path = (
    get_second_note[2],
    [],
    Field.POI.Coordinates.Notes.Wing.kLeft.withRotation(0),
)

get_fourth_note: path = (
    get_third_note[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withRotation(0),
)

go_to_midline_1: path = (
    get_fourth_note[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kFarLeft.withOffset(Translation2d(-2.3, -1))
)

get_fifth_note: path = (
    go_to_midline_1[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kMidLeft.withRotation(0)
)

go_to_midline_2: path = (
    get_fifth_note[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kMidLeft.withOffset(Translation2d(-2.3, 1)).withRotation(math.radians(30))
)
