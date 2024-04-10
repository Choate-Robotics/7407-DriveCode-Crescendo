from units.SI import meters, radians
import math
import constants

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial = (constants.field_length/2, constants.field_width/2, 0)

drive_forward = (
    (initial[0], initial[1], 0),
    [],
    (initial[0] + 2, initial[1], 0),
)

drive_left = (
    (initial[0] + 2, initial[1], math.pi/2),
    [],
    (initial[0] + 2, initial[1] + 2, math.pi/2)
)

drive_back = (
    (initial[0] + 2, initial[1] + 2, 0),
    [],
    (initial[0], initial[1] + 2, 0)
)

drive_right = (
    (initial[0], initial[1] + 2, math.pi/2),
    [],
    (initial[0], initial[1], math.pi/2)
)
