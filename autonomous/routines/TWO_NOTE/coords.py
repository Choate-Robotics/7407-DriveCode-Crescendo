from units.SI import meters, radians
from robot_systems import Field

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (1.9, 5.5, 0)

get_first_ring: path = (
    initial,
    [],
    (Field.POI.Notes.Wing.kCenter.x, Field.POI.Notes.Wing.kCenter.y, 0),
)

get_second_ring: path = (
    get_first_ring[2],
    [],
    (Field.POI.Notes.Wing.kLeft.x, Field.POI.Notes.Wing.kLeft.y, 0),
)

leave: path = (
    get_second_ring[2],
    [],
    (Field.POI.Notes.MidLine.kMid_left.x, Field.POI.Notes.MidLine.kMid_left.y, 0),
)
