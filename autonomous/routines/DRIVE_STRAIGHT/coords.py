from units.SI import meters, radians
import constants
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial = (constants.field_length/2, constants.field_width/2, math.radians(180))

drive_forward = (
    initial,
    [],
    (initial[0] + 7, initial[1], math.radians(180)),
)


