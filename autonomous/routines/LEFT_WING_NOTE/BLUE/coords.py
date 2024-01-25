from units.SI import meters, radians
import constants

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (0, 7.5, 0)

drive_to_note: path = (
    initial,
    [],
    (constants.BlueWingNotePositionDict["left"][0], constants.BlueWingNotePositionDict["left"][1], 0)
)

drive_back: path = (
    drive_to_note[2],
    [],
    initial
)