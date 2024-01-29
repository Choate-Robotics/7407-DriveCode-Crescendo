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

field_width = 8.21
field_length = 16.54
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
        
        amp_y = field_width
        
        amp_y_robot = amp_y - drivetrain_length_with_bumpers / 2
        
        amp_x = 72.6 * inches_to_meters
        
        amp_rotation = Rotation2d(math.radians(90))
        
    class Stage:
        
        stage_x = 231.20 * inches_to_meters # center stage x
        
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
elevator_gear_ratio: float = 0.7 # TODO: PLACEHOLDER
elevator_driver_gear_circumference: float = 0.5 # TODO: PLACEHOLDER
elevator_length: float = 0.55 # REAL VALUE: Meters
elevator_max_length: float = 10.7 # TODO: PLACEHOLDER
  
# INTAKE
intake_inner_gear_ratio = 15 / 1 #TODO: placeholder
intake_outer_gear_ratio = 20 / 1 #TODO: placeholder

# WRIST
wrist_gear_ratio = 3 

wrist_time_to_max_vel = 0.3