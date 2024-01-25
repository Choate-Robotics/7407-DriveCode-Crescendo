from units.SI import meters, radians
import utils

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (0, 7.5, 0)

drive_to_note: path = (
    initial,
    [],
    (utils.POI.Notes.Wing.kRight.x, utils.POI.Notes.Wing.kRight.y, 0)
)

drive_back: path = (
    drive_to_note[2],
    [],
    initial
)