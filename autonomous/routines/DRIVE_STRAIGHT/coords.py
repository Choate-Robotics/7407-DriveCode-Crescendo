from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (0, 0, 0)

drive_forward: path = (
    initial,
    [],
    (initial[0] + 0.2, initial[1], 0),
)


