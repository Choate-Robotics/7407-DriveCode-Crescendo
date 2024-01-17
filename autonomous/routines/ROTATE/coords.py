from units.SI import meters, radians
from command.autonomous import RotateInPlace

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial: coord = (0, 0, 0)

start_rotating: path = (
    initial,
    [],
    (initial[0], initial[1], 3.14),
)