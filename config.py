# from dataclasses import dataclass
from enum import Enum

from wpilib import AnalogEncoder, DigitalInput
from wpimath.geometry import Pose3d, Rotation3d

from toolkit.motors import SparkMaxConfig

comp_bot: DigitalInput = DigitalInput(
    0
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



#intake
inner_intake_id = 10 #placeholder
outer_intake_front_id = 11 #placeholder
outer_intake_back_id = 12 #placeholder
deploy_intake_id = 13 #placeholder
intake_beam_break_channel = 1 #placeholder


intake_inner_speed = 0.25  # placeholder
intake_outer_speed = 0.5  # placeholder
intake_outer_idle_speed = 0.25  # placeholder


intake_timeout = 5 #placeholder
intake_roller_current_limit = 1 #placeholder
intake_deploy_current_limit = 1 #placeholder
tenting_deploy_current_limit = 1 #placeholder
intake_sensor_debounce = 0.2 #placeholder


# elevator
elevator_can_id: int = 73  # TODO: PLACEHOLDER
elevator_can_id_2: int = 74  # TODO: PLACEHOLDER
elevator_ramp_rate: float = 1.0  # TODO: PLACEHOLDER
elevator_max_rotation: float = 1.0  # TODO: PLACEHOLDER
elevator_auto_position: float = 1.0  # TODO: PLACEHOLDER
elevator_feed_forward: float = 0.65  # TODO: PLACEHOLDER

# Wrist
wrist_motor_id = 30
WRIST_CONFIG = SparkMaxConfig(0.1, 0, 0.003, 0.00015, (-0.5, 0.5))
feed_motor_id = 50
FEED_CONFIG = SparkMaxConfig(0.1, 0, 0.003, 0.00015, (-0.5, 0.5))
feeder_velocity = 132
feeder_pass_velocity = 5
wrist_stage_max:radians = 0 #TODO: PLACEHOLDER


# LEDS
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
    elevator_down = Pose3d(0, 0, 0, Rotation3d(0, 0, 0))
    elevator_up = Pose3d(0, 0, 0, Rotation3d(0, 0, 0))


# DRIVETRAIN
front_left_move_id = 4
front_left_turn_id = 5
front_left_encoder_port = AnalogEncoder(0)
front_left_encoder_zeroed_pos = 0.678 if comp_bot.get() else 0.0

front_right_move_id = 6
front_right_turn_id = 7
front_right_encoder_port = AnalogEncoder(2)
front_right_encoder_zeroed_pos = 0.503 if comp_bot.get() else 0.0

back_left_move_id = 2
back_left_turn_id = 3
back_left_encoder_port = AnalogEncoder(1)
back_left_encoder_zeroed_pos = 0.964 if comp_bot.get() else 0.0

back_right_move_id = 8
back_right_turn_id = 9
back_right_encoder_port = AnalogEncoder(3)
back_right_encoder_zeroed_pos = 0.260 if comp_bot.get() else 0.0

driver_centric: bool = True
drivetrain_reversed: bool = False



# Gyro
gyro_id = 20

# Elevator
elevator_moving = False
elevator_stage_max:meters = 0.1 #meters


# c = drag coefficient
# a = projectile area (m^2)
# m = projectile mass (kg)
# rho_air = air density (kg/m^3)
# g = acceleration due to gravity (m/s^2)
# v0 = initial velocity of shooter flywheel (m/s) config
# delta_x = distance from shooter to target (COULD BE IN ODOMETRY) (m)
# y = height of target (COULD BE IN ODOMETRY) (m) const
# tol = tolerance of error in distance to target (m)




#Flywheel
flywheel_id_1 = 1 #TODO: placeholder
flywheel_id_2 = 2 #TODO: placeholder
flywheel_motor_count = 2 #TODO: placeholder
v0_flywheel = 15 #TODO: placeholder
shooter_tol = 0.001 # For aim of shooter
max_sim_times = 100 # To make sure that we don't have infinite while loop
