from units.SI import meters, radians, inches_to_meters
from robot_systems import Field
from wpimath.geometry import Translation2d
import constants
import math

from command.autonomous.trajectory import CustomTrajectory, PoseType

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

initial = (1.9 - constants.drivetrain_length_with_bumpers/2, constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap, math.radians(-180))