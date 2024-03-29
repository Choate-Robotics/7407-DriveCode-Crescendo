from units.SI import meters, radians
from robot_systems import Field
from utils import POIPose

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (1.9, 5.5, 0)

get_first_ring: path = (
    initial,
    [],
    Field.POI.Coordinates.Notes.Wing.kCenter.withRotation(0),
)

get_second_ring: path = (
    get_first_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kMidLeft.withRotation(0),
)

leave: path = (
    get_second_ring[2],
    [],
    Field.POI.Coordinates.Notes.MidLine.kMidRight.withRotation(0),
)
