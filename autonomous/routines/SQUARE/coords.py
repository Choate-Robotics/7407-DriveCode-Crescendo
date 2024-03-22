from units.SI import meters, radians
import math
import constants

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (constants.field_length/2, constants.field_width/2, 0)

drive_forward: path = (
    (initial[0], initial[1], math.pi/2),
    [],
    (initial[0], initial[1] + 1, math.pi/2),
)

drive_left: path = (
    (initial[0], initial[1] + 1, 0),
    [],
    (initial[0] + 1, initial[1] + 1, 0)
)

drive_back: path = (
    (initial[0] + 1, initial[1] + 1, math.pi/2),
    [],
    (initial[0] + 1, initial[1], math.pi/2)
)

drive_right: path = (
    (initial[0] + 1, initial[1], 0),
    [],
    (initial[0], initial[1], 0)
)
