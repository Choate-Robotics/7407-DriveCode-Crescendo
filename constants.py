from units.SI import rotations, rotations_per_minute, meters, inches_to_meters, feet_to_meters, degrees_per_second__to__radians_per_second
import math

# DRIVETRAIN
drivetrain_turn_gear_ratio = 18.4
drivetrain_wheel_gear_ratio = 6.12

track_width = 14 * inches_to_meters

drivetrain_length = 25 * inches_to_meters

bumper_thickness = 1.5 * inches_to_meters

drivetrain_length_with_bumpers = drivetrain_length + (2 * bumper_thickness)

drivetrain_max_vel = 10 * feet_to_meters
drivetrain_max_accel = 5 * feet_to_meters

drivetrain_max_angular_vel = 50 * degrees_per_second__to__radians_per_second

drivetrain_move_motor_free_speed: rotations_per_minute = 6300 # 5676 is the free speed RPM of the NEO


drivetrain_wheel_diameter: meters = 4 * inches_to_meters  # 3.5 is the diameter of the wheel in inches


drivetrain_move_gear_ratio: rotations_per_minute = drivetrain_move_motor_free_speed / drivetrain_wheel_gear_ratio # is the RPM constant multiple of the driving motor

#TODO: Change this
# the below variable is the rotation the motor rotates per meter of wheel movement
drivetrain_move_gear_ratio_as_rotations_per_meter: float = (1 / (drivetrain_wheel_diameter * math.pi)) * drivetrain_wheel_gear_ratio


