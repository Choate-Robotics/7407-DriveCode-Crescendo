from units.SI import meters, radians, feet_to_meters
import constants
import math

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial = (constants.field_length/2, constants.field_width/2, math.radians(180))

drive_forward = (
    initial,
    [],
    (initial[0] + 10*feet_to_meters, initial[1], math.radians(180)),
)

# drive_forward_2 = (
#     drive_forward[2],
#     [],
#     (initial[0] + 7, initial[1], math.radians(180))
# )


