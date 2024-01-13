from units.SI import rotations, rotations_per_minute, meters, inches_to_meters
import math

# DRIVETRAIN
drivetrain_turn_gear_ratio = 1
drivetrain_move_gear_ratio = 1

track_width = 0.0

drivetrain_max_vel = 0.0
drivetrain_max_accel = 0.0

drivetrain_max_angular_vel = 0.0

drivetrain_move_motor_free_speed: rotations_per_minute = 5676 # 5676 is the free speed RPM of the NEO

drivetrain_turn_gear_ratio: rotations = 12.8 # 12.8 is the gear ratio of the turn motor


drivetrain_wheel_diameter: meters = 4 * inches_to_meters  # 3.5 is the diameter of the wheel in inches


drivetrain_move_gear_ratio: rotations_per_minute = drivetrain_move_motor_free_speed / drivetrain_move_gear_ratio # is the RPM constant multiple of the driving motor

#TODO: Change this
# the below variable is the rotation the motor rotates per meter of wheel movement
drivetrain_move_gear_ratio_as_rotations_per_meter = (1 / (drivetrain_wheel_diameter * math.pi)) * drivetrain_move_gear_ratio


