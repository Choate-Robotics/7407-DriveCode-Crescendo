from units.SI import meters, radians
from robot_systems import Field

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (0, 7.5, 0)

drive_to_note: path = (
    initial,
    [],
    (Field.POI.Notes.Wing.kRight.x, Field.POI.Notes.Wing.kRight.y, 0)
)

drive_back: path = (
    drive_to_note[2],
    [],
    initial
)