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
a = 14*0.0254*2*0.0254 # projectile surface area (m^2)
m = 0.235301 # projectile mass (kg)
rho_air = 1.28 # air density (kg/m^3)
g = 9.8 # acceleration due to gravity (m/s^2)
speaker_z = 1.7 # height of target (m) CHANGE THIS
speaker_location = Translation2d(0, 0)
from units.SI import rotations, rotations_per_minute, meters, inches_to_meters, feet_to_meters, \
    degrees_per_second__to__radians_per_second
import math
from wpimath.geometry import Pose2d, Rotation2d


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


# the below variable is the rotation the motor rotates per meter of wheel movement
drivetrain_move_gear_ratio_as_rotations_per_meter: float = (1 / (drivetrain_wheel_diameter * math.pi)) * drivetrain_wheel_gear_ratio


# Field

field_width = 8.21 # meters
field_length = 16.54 # meters
class FieldPos:
    pose_reverse = Rotation2d(math.radians(180))

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
        
        speaker_z_bottom = 78.13 * inches_to_meters
        
        speaker_z = (speaker_z_top + speaker_z_bottom) / 2

        amp_y = field_width

        amp_y_robot = amp_y - drivetrain_length_with_bumpers / 2

        amp_x = 72.6 * inches_to_meters

        amp_rotation = Rotation2d(math.radians(90))

    class Stage:
        
        stage_x = 231.20 * inches_to_meters  # center stage x

        post_deviation = .25

        stage_length = 106.19 * inches_to_meters

        stage_width = 122.62 * inches_to_meters

        stage_y = field_width / 2

        tag_line_spacing = 22 * inches_to_meters

        line_x_and_y = math.sin(math.radians(45)) * tag_line_spacing

        y_deviation = stage_y - 146.19 * inches_to_meters + line_x_and_y

        x_deviation = stage_x - 182.73 * inches_to_meters + line_x_and_y

        left_rotation = Rotation2d(math.radians(300))

        right_rotation = Rotation2d(math.radians(60))

    class Source:
        source_x = field_length - 86.1 * inches_to_meters / 2
        source_y = 50.75 * inches_to_meters
        rotation = Rotation2d(math.radians(-240))

# ELEVATOR
elevator_gear_ratio: float = 25 # REAL VALUE: 25:1 gear ratio
elevator_driver_gear_circumference: float = math.pi * 1 * inches_to_meters # REAL VALUE: Meters
elevator_length: float = 0.55 # REAL VALUE: Meters
elevator_max_length: float = 21 * inches_to_meters # REAL VALUE: Meters
  
# INTAKE
intake_inner_gear_ratio = 15 / 1 #TODO: placeholder
intake_outer_gear_ratio = 20 / 1 #TODO: placeholder

# WRIST
wrist_gear_ratio = 3

wrist_time_to_max_vel = 0.3
