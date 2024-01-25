from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (0, 7.57, 0)

left_wing_note: path = (
    initial,
    [],
    (2.9, 7, 0)
)