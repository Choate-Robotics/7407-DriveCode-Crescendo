# Imports
import math
from wpimath.geometry import Rotation2d, Translation2d

import config
from units.SI import (
    degrees_per_second__to__radians_per_second,
    feet_to_meters,
    inches_to_meters,
    meters,
    rotations_per_minute,
    degrees_to_radians,
    meters_per_second_squared,
    kilograms,
)

# Main


# c = drag coefficient
# a = projectile sectional area (m^2)
# m = projectile mass (kg)
# rho_air = air density (kg/m^3)
# g = acceleration due to gravity (m/s^2)
c = 0.47  # drag coefficient
a = 14 * 0.0254 * 2 * 0.0254  # projectile surface area (m^2)
m: kilograms = 0.235301  # projectile mass (kg)
rho_air = 1.28  # air density (kg/m^3)
g: meters_per_second_squared = 9.8  # acceleration due to gravity (m/s^2)

# Drivetrain
drivetrain_turn_gear_ratio = 150 / 7
drivetrain_wheel_gear_ratio = 5.9
track_width = 20 * inches_to_meters  # TODO: change to 20 inches for actual robot
drivetrain_length = 25 * inches_to_meters
bumper_thickness = 3.5 * inches_to_meters
drivetrain_length_with_bumpers = drivetrain_length + (2 * bumper_thickness)

drivetrain_move_motor_free_speed: rotations_per_minute = (
    6000  # 5676 is the free speed RPM of the NEO, 6000 is the free speed RPM of the Kraken without FOC
)

drivetrain_wheel_diameter: meters = (
        3.86 * inches_to_meters
)  
 

drivetrain_max_vel:meters = (
    ((drivetrain_move_motor_free_speed / 60) / drivetrain_wheel_gear_ratio) * (drivetrain_wheel_diameter * math.pi)
    )
# drivetrain_max_vel = 17.7
drivetrain_max_accel: meters_per_second_squared = 0 # setting to 0 will set to default motor accel
drivetrain_max_angular_vel = 720 * degrees_per_second__to__radians_per_second
drivetrain_max_angular_accel = 720 * degrees_per_second__to__radians_per_second


# the below variable is the rotation the motor rotates per meter of wheel movement
drivetrain_move_gear_ratio_as_rotations_per_meter: float = (
    1 / (drivetrain_wheel_diameter * math.pi)
) * drivetrain_wheel_gear_ratio

# Field
field_width = 8.21  # meters
field_length = 16.54  # meters


class FieldPos:
    pose_reverse = Rotation2d(math.radians(180))

    wing_boundary = 231 * inches_to_meters
    
    op_wing_boundary = field_length - wing_boundary

    # all poses are relative to the blue field origin
    class Wing:
        note_x = 114 * inches_to_meters

        note_gap = 57 * inches_to_meters

        note_init = field_width / 2

    class MidLine:
        note_init = 29.64 * inches_to_meters

        note_gap = 66 * inches_to_meters

        mid_line = field_length / 2

    class Scoring:
        speaker_y = 218.42 * inches_to_meters

        speaker_z_top = 82.90 * inches_to_meters

        speaker_depth = 18 * inches_to_meters

        speaker_z_bottom = 78.13 * inches_to_meters

        speaker_z = speaker_z_top #(speaker_z_top + speaker_z_bottom) / 2

        speaker_x = speaker_depth / 2.5

        amp_y = field_width

        amp_y_robot = amp_y - drivetrain_length_with_bumpers / 2

        amp_x = 72.6 * inches_to_meters

        amp_rotation = Rotation2d(math.radians(90))

    class Stage:
        stage_x = 231.20 * inches_to_meters  # center stage x

        post_deviation = 0.25

        stage_length = 106.19 * inches_to_meters

        stage_width = 122.62 * inches_to_meters

        stage_y = field_width / 2

        tag_line_spacing = 22 * inches_to_meters

        line_x_and_y = math.sin(math.radians(45)) * tag_line_spacing

        y_deviation = stage_y - 146.19 * inches_to_meters + line_x_and_y

        x_deviation = stage_x - 182.73 * inches_to_meters + line_x_and_y

        left_rotation = Rotation2d(math.radians(180-300))

        right_rotation = Rotation2d(math.radians(180-60))

    class Source:
        source_x = field_length - 86.1 * inches_to_meters / 2
        source_y = 50.75 * inches_to_meters
        rotation = Rotation2d(math.radians(-240))


# Elevator
elevator_gear_ratio: float = 25 / 2  # REAL VALUE: 25:1 gear ratio, divide by 2 to get top stage
elevator_driver_gear_circumference: meters = (
        math.pi * 1.79 * inches_to_meters
)  # REAL VALUE: Meters
elevator_max_length: meters = 21.653 * inches_to_meters  # REAL VALUE: Meters
elevator_bottom_total_height: meters = 26.25 * inches_to_meters
elevator_max_length_stage: meters = 0 * inches_to_meters

# INTAKE
intake_inner_gear_ratio = 36 / 11  #REAL VALUE: 36:11 gear ratio
intake_outer_gear_ratio = 58 / 12  #REAL VALUE: 58:12 gear ratio
intake_deploy_gear_ratio = 20 * (32 / 14)  #REAL VALUE: 20:1 * 32:14 gear ratio

# LIMELIGHT
limelight_height = 26 * inches_to_meters
limelight_height_LL3 = 26.17 * inches_to_meters
limelight_right = -10.7255 * inches_to_meters
limelight_right_LL3 = -10.6338 * inches_to_meters
limelight_forward_LL3 = 10.186 * inches_to_meters
limelight_forward = 2.395 * inches_to_meters
limelight_elevator_angle = 25 * degrees_to_radians
limelight_back_yaw = 180 * degrees_to_radians

# WRIST
wrist_refrence_frame: float = 95.95 / 100
wrist_gear_ratio: float = 48 * wrist_refrence_frame  # REAL VALUE: 48:1 gear ratio muahhaha

# Flywheel
flywheel_mass = 0.85  # kilograms
flywheel_shaft_mass = .127  # kilograms
flywheel_radius_outer = 2 * inches_to_meters
flywheel_shaft_radius = 0.5 * inches_to_meters
flywheel_gear_ratio = 1 / 1  #REAL VALUE: 1:1 gear ratio
shooter_height = 21 * inches_to_meters  # REAL VALUE: Meters
shooter_offset_y = 6 * inches_to_meters  # REAL VALUE: Meters
NEO_MAX_RPM = (5345 + 5686) / 2
wrist_max_rotation = 60 * degrees_to_radians
wrist_min_rotation = -44 * degrees_to_radians
wrist_min_rotation_stage = 22 * degrees_to_radians

# Pathing
post_avoidance_distance = 0.5
