import constants
from units.SI import meters, radians
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (0, 0, 0)

drive_to_note: path = (
    initial,
    [],
    (constants.BlueWingNotePositionDict["right"][0], constants.BlueWingNotePositionDict["right"][1], 0),
)

drive_back: path = (
    drive_to_note[2],
    [],
    initial
)
