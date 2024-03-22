# from dataclasses import dataclass
import math
from enum import Enum

import rev

# from rev import CANSparkMax
from wpilib import AnalogEncoder, DigitalInput
from wpimath.geometry import Pose3d, Rotation3d

import constants
from toolkit.motors import SparkMaxConfig
from toolkit.motors.ctre_motors import TalonConfig
from units.SI import (
    degrees_to_radians,
    inches_to_meters,
    meters,
    meters_per_second,
    radians,
    degrees
)

# from typing import Literal


comp_bot: DigitalInput = DigitalInput(
    2
)  # if false, we are using the practice bot (we will put a jumper on the DIO port)


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

period: float = 0.04  # seconds

# Giraffe
elevator_wrist_limit: float = 0.75  # TODO: PLACEHOLDER
elevator_wrist_threshold: float = 0.75  # TODO: PLACEHOLDER

# odometry config

odometry_debounce: float = 0.1  # TODO: PLACEHOLDER
stage_distance_threshold: float = constants.FieldPos.Stage.stage_length * math.sin(
    math.radians(30)
)


# STATE VARIABLES -- PLEASE DO NOT CHANGE


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


active_team: Team = Team.RED


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
    init_elevator_front = Pose3d(
        constants.limelight_right_LL3,
        constants.limelight_forward_LL3,
        constants.limelight_height_LL3,
        Rotation3d(0, constants.limelight_elevator_angle, 0),
    )
    init_elevator_back = Pose3d(
        constants.limelight_right,
        constants.limelight_forward,
        constants.limelight_height,
        Rotation3d(0, constants.limelight_elevator_angle, constants.limelight_back_yaw),
    )
    fixed_intake = Pose3d(0, 0, 0, Rotation3d(0, 0, 0))


# Intake
inner_intake_id = 13
outer_intake_back_id = 17
deploy_intake_id = 12

intake_inner_speed = 0.5
intake_inner_pass_speed = 0.1
intake_inner_eject_speed = 1
intake_outer_speed = 1
intake_outer_idle_speed = 0
intake_outer_eject_speed = 1

deploy_intake_timeout = 0.25
deploy_tenting_timeout = 0.1

intake_timeout = 5
intake_roller_current_limit = 18
intake_deploy_current_limit = 30
tenting_deploy_current_limit = 30
intake_sensor_debounce = 0.1
intake_distance_sensor_threshold: float = 0.3

double_note_timeout = 2

# Elevator

elevator_can_id: int = 10
elevator_can_id_2: int = 15
elevator_ramp_rate: float = 0.0
elevator_feed_forward: float = 0.0
elevator_climb_ff: float = -1
elevator_climb_current_limit: float = 45
elevator_zeroed_pos = 0.036 if comp_bot.get() else 0.023 
#helloworld
# Wrist
wrist_zeroed_pos = 0.0
wrist_motor_id = 2
wrist_time_to_max_vel = 0.0
feed_motor_id = 3
feed_motor_ramp_rate = 0
wrist_max_ff = -0.32  # used to be -0.9
wrist_ff_offset = 14.3 * degrees_to_radians
stage_timeout = 5
wrist_tent_limit = 15 * degrees_to_radians
feeder_velocity = 0.2
feeder_voltage_feed = 8
feeder_voltage_trap = 14
feeder_voltage_crawl = 4.15
feeder_pass_velocity = 0.5
feeder_pass_voltage = 2
feeder_sensor_threshold = 0.65
feeder_beam_break_first_channel = 1
feeder_beam_break_second_channel = 0

# DRIVETRAIN
front_left_move_id = 7
front_left_turn_id = 8
front_left_encoder_port = AnalogEncoder(3)
front_left_encoder_zeroed_pos = 0.487 if comp_bot.get() else 0.860

front_right_move_id = 4
front_right_turn_id = 6
front_right_encoder_port = AnalogEncoder(2)
front_right_encoder_zeroed_pos = 0.793 if comp_bot.get() else 0.536

back_left_move_id = 11
back_left_turn_id = 14
back_left_encoder_port = AnalogEncoder(1 if comp_bot.get() else 0)
back_left_encoder_zeroed_pos = 0.221 if comp_bot.get() else 0.458

back_right_move_id = 18
back_right_turn_id = 16
back_right_encoder_port = AnalogEncoder(0 if comp_bot.get() else 1)
back_right_encoder_zeroed_pos = 0.151 if comp_bot.get() else 0.984
driver_centric: bool = True
drivetrain_reversed: bool = False

drivetrain_rotation_P: float = 8
drivetrain_rotation_I: float = 0.0
drivetrain_rotation_D: float = 0.08
drivetrain_aiming_max_angular_speed: radians = 50#constants.drivetrain_max_angular_vel
drivetrain_aiming_max_angular_accel: radians = 35 #constants.drivetrain_max_angular_accel
drivetrain_aiming_offset: degrees = 2.0 # degrees
drivetrain_rotation_enable_tuner: bool = True

# Flywheel
flywheel_id_1 = 19
flywheel_id_2 = 1
flywheel_motor_count = 1
flywheel_amp_speed: meters = 19.5
flywheel_distance_scalar: float = 1.8
v0_flywheel_minimum: meters_per_second = 14
v0_flywheel_maximum: meters_per_second = 28
# v0_effective_flywheel: meters_per_second = 12
idle_flywheel: meters_per_second = v0_flywheel_minimum / 2
shooter_tol = 0.001  # For aim of shooter
max_sim_times = 100  # To make sure that we don't have infinite while loop
auto_shoot_deadline = 1.2
auto_intake_note_deadline = 3
auto_path_intake_note_deadline = 0.5

flywheel_feed_forward = 1 / constants.NEO_MAX_RPM  # TODO: placeholder
flywheel_shot_tolerance: meters_per_second = 0.5
flywheel_shot_current_threshold = 20


# Odometry
odometry_visible_tags_threshold = 2
odometry_tag_area_threshold = 0
odometry_vision_deviation_threshold = 0.5
odometry_tag_distance_threshold: meters = 4
odometry_two_tag_distance_threshold = 7
odometry_distance_deviation_threshold: meters = 0.5
odometry_std_auto_formula = lambda x: abs(x**2) / 2.5  # noqa
odometry_std_tele_formula = lambda x: abs(x**1.3) / 1.3  # noqa

# Configs
ELEVATOR_CONFIG = SparkMaxConfig(  # -.65, 1
    0.2,
    0.0,
    0.08,
    elevator_feed_forward,
    (-0.75, 1),
    idle_mode=rev.CANSparkMax.IdleMode.kBrake,
)

ELEVATOR_CLIMB_CONFIG = SparkMaxConfig(
    100,
    0.0,
    0,
    elevator_feed_forward,
    (-0.6, 0.5),
    idle_mode=rev.CANSparkMax.IdleMode.kBrake,
)
# P=.2, I=0, D=0.003
WRIST_CONFIG = SparkMaxConfig(
    0.4, 0, 40, 0, (-0.5, 0.5), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


FEED_CONFIG = SparkMaxConfig(0.08, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)

INNER_CONFIG = SparkMaxConfig(0.08, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)

OUTER_CONFIG = SparkMaxConfig(0.5, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)

DEPLOY_CONFIG = SparkMaxConfig(0.5, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)

# FLYWHEEL_CONFIG = SparkMaxConfig(
#     0.0005,
#     0.0,
#     0.0003,
#     flywheel_feed_forward,
#     idle_mode=rev.CANSparkMax.IdleMode.kCoast,
# )
FLYWHEEL_CONFIG = TalonConfig(
    0.5, 0, 0, 0, 0, brake_mode=False, current_limit=60, kV=0.12
)


TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)

MOVE_CONFIG = TalonConfig(
    0.11,
    0,
    0,
    0.25,
    0.01,
    brake_mode=True,
    current_limit=70,  # integral_zone=1000, max_integral_accumulator=10000
)

# Giraffe

staging_angle: radians = 60 * degrees_to_radians


class Giraffe:
    class GiraffePos:
        class Special(Enum):
            kStage = 0
            kAim = 1
            kHeightAuto = 2
            kCurrentAngle = 3
            kCurrentHeight = 4

        def __init__(self, height: meters | Special, wrist_angle: radians | Special):
            self.height = height
            self.wrist_angle = wrist_angle

    kIdle = GiraffePos(0, staging_angle)

    kStage = GiraffePos(0, GiraffePos.Special.kStage)

    kAim = GiraffePos(GiraffePos.Special.kCurrentAngle, GiraffePos.Special.kAim)

    kAimLow = GiraffePos(0, GiraffePos.Special.kAim)

    kAimHigh = GiraffePos(constants.elevator_max_length, GiraffePos.Special.kAim)

    kClimbReach = GiraffePos(constants.elevator_max_length, 10 * degrees_to_radians)

    kClimbPullUp = GiraffePos(0, 50 * degrees_to_radians)

    kTestFF = GiraffePos(0, 20 * degrees_to_radians)

    kClimbTrap = GiraffePos(constants.elevator_max_length, 30 * degrees_to_radians)

    kAmp = GiraffePos(0.27 + 2 * inches_to_meters, -25 * degrees_to_radians)

    kElevatorHigh = GiraffePos(
        constants.elevator_max_length, GiraffePos.Special.kCurrentAngle
    )

    kElevatorLow = GiraffePos(0, GiraffePos.Special.kCurrentAngle)

    kElevatorMid = GiraffePos(
        constants.elevator_max_length / 2, GiraffePos.Special.kCurrentAngle
    )


# """
# c = drag coefficient
# a = projectile area (m^2)
# m = projectile mass (kg)
# rho_air = air density (kg/m^3)
# g = acceleration due to gravity (m/s^2)
# v0 = initial velocity of shooter flywheel (m/s) config
# delta_x = distance from shooter to target (COULD BE IN ODOMETRY) (m)
# y = height of target (COULD BE IN ODOMETRY) (m) const
# tol = tolerance of error in distance to target (m)
# """

# Gyro
gyro_id = 29

driver_rumble_intensity:float = 1.0
driver_rumble_time:float= 5

operator_rumble_intensity:float = 1.0
operator_rumble_time:float= 5