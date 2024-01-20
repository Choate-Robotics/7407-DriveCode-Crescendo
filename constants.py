from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d
"""
c = drag coefficient
a = projectile sectional area (m^2)
m = projectile mass (kg)
rho_air = air density (kg/m^3)
g = acceleration due to gravity (m/s^2)
v0 = initial velocity of shooter flywheel (m/s) config
delta_x = distance from shooter to target (COULD BE IN ODOMETRY) (m)
y = height of target (COULD BE IN ODOMETRY) (m) const
tol = tolerance of error in distance to target (m)
"""

c = 0.47 # drag coefficient
a = 14*0.0254*2*0.0254 # projectile volume (m^3)
m = 0.235301 # projectile mass (kg)
rho_air = 1.28 # air density (kg/m^3)
g = 9.8 # acceleration due to gravity (m/s^2)
speaker_z = 1.7 # height of target (m) CHANGE THIS
speaker_location = Translation2d(0, 0)
from units.SI import rotations, rotations_per_minute, meters, inches_to_meters, feet_to_meters, \
    degrees_per_second__to__radians_per_second
import math

# DRIVETRAIN
drivetrain_turn_gear_ratio = 18.4
drivetrain_wheel_gear_ratio = 6.12

track_width = 14 * inches_to_meters  # TODO: change to 20 inches for actual robot

drivetrain_length = 25 * inches_to_meters

bumper_thickness = 1.5 * inches_to_meters

drivetrain_length_with_bumpers = drivetrain_length + (2 * bumper_thickness)

drivetrain_max_vel = 80 * feet_to_meters
drivetrain_max_accel = 5 * feet_to_meters

drivetrain_max_angular_vel = 500 * degrees_per_second__to__radians_per_second

drivetrain_move_motor_free_speed: rotations_per_minute = 6300  # 5676 is the free speed RPM of the NEO

drivetrain_wheel_diameter: meters = 4 * inches_to_meters  # 3.5 is the diameter of the wheel in inches

drivetrain_move_gear_ratio: rotations_per_minute = drivetrain_move_motor_free_speed / drivetrain_wheel_gear_ratio  # is the RPM constant multiple of the driving motor

# TODO: Change this
# the below variable is the rotation the motor rotates per meter of wheel movement
drivetrain_move_gear_ratio_as_rotations_per_meter: float = (1 / (
            drivetrain_wheel_diameter * math.pi)) * drivetrain_wheel_gear_ratio

# ELEVATOR
elevator_gear_ratio: float = 0.7  # TODO: PLACEHOLDER
elevator_driver_gear_circumference: float = 0.5  # TODO: PLACEHOLDER
elevator_length: float = 0.55  # REAL VALUE: Meters
