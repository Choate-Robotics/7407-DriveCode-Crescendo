from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (0, 0, 0)

drive_forward_1: path = (
    initial,
    [],
    (initial[0] + 1, initial[1], 0),
)

drive_forward_2: path = (
    drive_forward_1[2],
    [],
    (initial[0]+1, initial[1]+1, 0),
)

# drive_forward_3: path = (
#     initial,
#     [],
#     (initial[0] + 1, initial[1], 0),
# )

# drive_forward_4: path = (
#     initial,
#     [],
#     (initial[0] + 1, initial[1], 0),
# )

