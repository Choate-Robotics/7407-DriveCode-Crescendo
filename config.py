
# from dataclasses import dataclass
from enum import Enum

from wpilib import AnalogEncoder, DigitalInput
from wpimath.geometry import Pose3d, Rotation3d

from toolkit.motors import SparkMaxConfig
from rev import CANSparkMax
import rev
from enum import Enum
import constants

from wpilib import AnalogEncoder, DigitalInput
from wpimath.geometry import Pose3d, Rotation3d

from toolkit.motors import SparkMaxConfig
from toolkit.motors.ctre_motors import TalonConfig


comp_bot: DigitalInput = DigitalInput(
    9
)  # if true, we are using the practice bot (we will put a jumper on the DIO port)

# from units.SI import (
#     inches_to_meters,
#     meters,
#     meters_per_second,
#     meters_per_second_squared,
#     radians,
# )

DEBUG_MODE: bool = True
# MAKE SURE TO MAKE THIS FALSE FOR COMPETITION
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
LOGGING: bool = True
LOG_OUT_LEVEL: int = 0
LOG_FILE_LEVEL: int = 1

# Levels are how much information is logged
# higher level = less information
# level 0 will log everything
# level 1 will log everything except debug
# and so on
# levels:
# 0 = All
# 1 = INFO
# 2 = WARNING
# 3 = ERROR
# 4 = SETUP
# anything else will log nothing
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

period: float = 0.03  # seconds




 


# Leds
def KRainbow():
    return {"type": 2}

class Type:
    def KStatic(r, g, b):
        return {"type": 1, "color": {"r": r, "g": g, "b": b}}

    def KTrack(r1, g1, b1, r2, g2, b2):
        return {
            "type": 3,
            "color": {"r1": r1, "g1": g1, "b1": b1, "r2": r2, "g2": g2, "b2": b2},
        }

    def KBlink(r, g, b):
        return {"type": 4, "color": {"r": r, "g": g, "b": b}}

    def KLadder(typeA, typeB, percent, speed):
        return {
            "type": 5,
            "percent": percent,  # 0-1
            "typeA": typeA,
            "typeB": typeB,
            "speed": speed,
        }

# TEAM
class Team(Enum):
    RED = 0
    BLUE = 1

active_team: Team = Team.BLUE

# LIMELIGHT
class LimelightPipeline:
    feducial = 0.0
    neural = 1.0
    retroreflective = 2.0

limelight_led_mode = {
    "pipline_default": 0,
    "force_off": 1,
    "force_blink": 2,
    "force_on": 3,
}

class LimelightPosition:
    init_elevator_front = Pose3d(0, 0, constants.limelight_height, Rotation3d(0, constants.limelight_elevator_angle, 0))
    init_elevator_back = Pose3d(0, 0, constants.limelight_height, Rotation3d(0, constants.limelight_elevator_angle, constants.limelight_back_yaw))
    fixed_intake = Pose3d(0,0,0, Rotation3d(0,0,0))

period: float = 0.03  # seconds

# Intake
inner_intake_id = 13
outer_intake_back_id = 17
deploy_intake_id = 12

intake_inner_speed = 0.25 
intake_outer_speed = 1 
intake_outer_idle_speed = .15

deploy_intake_timeout = .1 
deploy_tenting_timeout = .1

intake_timeout = 5
intake_roller_current_limit = 15
intake_deploy_current_limit = 30
tenting_deploy_current_limit = 30
intake_sensor_debounce = 0.2
intake_distance_sensor_threshold: float = 0.55

double_note_timeout = 2

# Elevator

elevator_can_id: int = 10
elevator_can_id_2: int = 15
elevator_ramp_rate: float = 1.0  # TODO: PLACEHOLDER
elevator_max_rotation: float = 1.0  # TODO: PLACEHOLDER
elevator_auto_position: float = 1.0  # TODO: PLACEHOLDER
elevator_feed_forward: float = 0.65  # TODO: PLACEHOLDER
elevator_moving = False
elevator_stage_max = 0.1  # meters
elevator_zeroed_pos = 0.0  # TODO: PLACEHOLDER: meters

# Wrist
wrist_zeroed_pos = 0.0
wrist_motor_id = 2
feed_motor_id = 3
wrist_flat_ff = -0.6 # TODO: FIND
feeder_velocity = .8
feeder_pass_velocity = 1
wrist_stage_max = 0  # TODO: PLACEHOLDER radians
feeder_sensor_threshold = .4

# DRIVETRAIN
front_left_move_id = 7
front_left_turn_id = 8
front_left_encoder_port = AnalogEncoder(3)
front_left_encoder_zeroed_pos = 0.678 if comp_bot.get() else 0.0

front_right_move_id = 5
front_right_turn_id = 6
front_right_encoder_port = AnalogEncoder(2)
front_right_encoder_zeroed_pos = 0.503 if comp_bot.get() else 0.0

back_left_move_id = 11
back_left_turn_id = 14
back_left_encoder_port = AnalogEncoder(1)
back_left_encoder_zeroed_pos = 0.964 if comp_bot.get() else 0.0

back_right_move_id = 19
back_right_turn_id = 16
back_right_encoder_port = AnalogEncoder(0)
back_right_encoder_zeroed_pos = 0.260 if comp_bot.get() else 0.0
driver_centric: bool = True
drivetrain_reversed: bool = False

# Flywheel
flywheel_id_1 = 19
flywheel_id_2 = 1
flywheel_motor_count = 1
v0_flywheel = 15  # TODO: placeholder
shooter_tol = 0.001  # For aim of shooter
max_sim_times = 100  # To make sure that we don't have infinite while loop
flywheel_feed_forward = 0.65  # TODO: placeholder

# Configs 
# TODO: PLACEHOLDER
ELEVATOR_CONFIG = SparkMaxConfig(
    0.055, 0.0, 0.01, elevator_feed_forward, (-.5, .75), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)
WRIST_CONFIG = SparkMaxConfig(0.1, 0, 0.003, 0.00015, (-0.5, 0.5),idle_mode=rev.CANSparkMax.IdleMode.kBrake)
FEED_CONFIG = SparkMaxConfig(0.0005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
INNER_CONFIG = SparkMaxConfig(.08, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
OUTER_CONFIG = SparkMaxConfig(.5, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
DEPLOY_CONFIG = SparkMaxConfig(.5, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
FLYWHEEL_CONFIG = SparkMaxConfig(
    0.055, 0.0, 0.01, flywheel_feed_forward, (-.5, .75), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = TalonConfig(
    0.11, 0, 0, 0.25, 0.01, brake_mode=True  # integral_zone=1000, max_integral_accumulator=10000
)


#Gyro
gyro_id = 29
 

