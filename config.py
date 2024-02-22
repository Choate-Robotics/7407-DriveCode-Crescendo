# from dataclasses import dataclass
from enum import Enum

from wpilib import AnalogEncoder, DigitalInput
from wpimath.geometry import Pose3d, Rotation3d

from toolkit.motors import SparkMaxConfig
from rev import CANSparkMax
import rev, math
from enum import Enum
import constants

from wpilib import AnalogEncoder, DigitalInput
from wpimath.geometry import Pose3d, Rotation3d

from toolkit.motors import SparkMaxConfig
from toolkit.motors.ctre_motors import TalonConfig
from units.SI import degrees_to_radians, meters, radians, meters_per_second
from typing import Literal

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

# Giraffe
elevator_wrist_limit: float = 0.75  # TODO: PLACEHOLDER
elevator_wrist_threshold: float = 0.75  # TODO: PLACEHOLDER

# odometry config

odometry_debounce: float = 0.1  # TODO: PLACEHOLDER
stage_distance_threshold: float = constants.FieldPos.Stage.stage_length * math.sin(math.radians(30))


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

# CLIMBING
climbing: bool = False


class LimelightPosition:
    init_elevator_front = Pose3d(constants.limelight_right_LL3, constants.limelight_forward_LL3,
                                 constants.limelight_height_LL3, Rotation3d(0, constants.limelight_elevator_angle, 0))
    init_elevator_back = Pose3d(constants.limelight_right, constants.limelight_forward, constants.limelight_height,
                                Rotation3d(0, constants.limelight_elevator_angle, constants.limelight_back_yaw))
    fixed_intake = Pose3d(0, 0, 0, Rotation3d(0, 0, 0))

# Intake
inner_intake_id = 13
outer_intake_back_id = 17
deploy_intake_id = 12

intake_inner_speed = 0.2
intake_inner_pass_speed = .1
intake_inner_eject_speed = 1
intake_outer_speed = 1
intake_outer_idle_speed = .15
intake_outer_eject_speed = 1

deploy_intake_timeout = .25
deploy_tenting_timeout = .1

intake_timeout = 5
intake_roller_current_limit = 18
intake_deploy_current_limit = 30
tenting_deploy_current_limit = 30
intake_sensor_debounce = 0.1
intake_distance_sensor_threshold: float = 0.5

double_note_timeout = 2

# Elevator

elevator_can_id: int = 10
elevator_can_id_2: int = 15
elevator_ramp_rate: float = .3
elevator_max_rotation: float = 1.0  # TODO: PLACEHOLDER
elevator_auto_position: float = 1.0  # TODO: PLACEHOLDER
elevator_feed_forward: float = 0.0  # TODO: PLACEHOLDER
elevator_moving = False
elevator_zeroed_pos = 0.023  # TODO: PLACEHOLDER: meters
#helloworld
# Wrist
wrist_zeroed_pos = 0.0
wrist_motor_id = 2
feed_motor_id = 3
wrist_flat_ff = -0.58  # TODO: FIND
stage_timeout = 5
wrist_tent_limit = 10 * degrees_to_radians
feeder_velocity = .2
feeder_voltage = 5.6
feeder_pass_velocity = .5
feeder_pass_voltage = 2
feeder_sensor_threshold = .65

# DRIVETRAIN
front_left_move_id = 7
front_left_turn_id = 8
front_left_encoder_port = AnalogEncoder(3)
front_left_encoder_zeroed_pos = 0.860 if comp_bot.get() else 0.860

front_right_move_id = 4
front_right_turn_id = 6
front_right_encoder_port = AnalogEncoder(2)
front_right_encoder_zeroed_pos = 0.536 if comp_bot.get() else 0.536

back_left_move_id = 11
back_left_turn_id = 14
back_left_encoder_port = AnalogEncoder(0)
back_left_encoder_zeroed_pos = 0.458 if comp_bot.get() else 0.458

back_right_move_id = 18
back_right_turn_id = 16
back_right_encoder_port = AnalogEncoder(1)
back_right_encoder_zeroed_pos = 0.984 if comp_bot.get() else 0.984
driver_centric: bool = True
drivetrain_reversed: bool = False

# Flywheel
flywheel_id_1 = 19
flywheel_id_2 = 1
flywheel_motor_count = 1
flywheel_amp_speed: meters = 5
v0_flywheel: meters_per_second = 22  # TODO: placeholder
shooter_tol = 0.001  # For aim of shooter
max_sim_times = 100  # To make sure that we don't have infinite while loop
flywheel_feed_forward = 0.65  # TODO: placeholder
flywheel_shot_tolerance: meters_per_second = 0.2
flywheel_shot_current_threshold = 20
# Configs 
# TODO: PLACEHOLDER
ELEVATOR_CONFIG = SparkMaxConfig(
    0.2, 0.0, 0.02, elevator_feed_forward, (-1, 1), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)
WRIST_CONFIG = SparkMaxConfig(1, 0, 0.002, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
FEED_CONFIG = SparkMaxConfig(0.08, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
INNER_CONFIG = SparkMaxConfig(.08, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
OUTER_CONFIG = SparkMaxConfig(.5, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
DEPLOY_CONFIG = SparkMaxConfig(.5, 0, 0, idle_mode=rev.CANSparkMax.IdleMode.kBrake)
FLYWHEEL_CONFIG = SparkMaxConfig(
    0.055, 0.0, 0.01, flywheel_feed_forward, (1, 1), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = TalonConfig(
    0.11, 0, 0, 0.25, 0.01, brake_mode=True  # integral_zone=1000, max_integral_accumulator=10000
)

# Giraffe

staging_angle:radians = 57.5 * degrees_to_radians


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

    kAim = GiraffePos(GiraffePos.Special.kHeightAuto, GiraffePos.Special.kAim)

    kAimLow = GiraffePos(0, GiraffePos.Special.kAim)

    kAimHigh = GiraffePos(constants.elevator_max_length, GiraffePos.Special.kAim)

    kClimbReach = GiraffePos(constants.elevator_max_length, 0)

    kClimbPullUp = GiraffePos(0, 0)

    kTestFF = GiraffePos(0, 20 * degrees_to_radians)

    kClimbTrap = GiraffePos(constants.elevator_max_length, 20 * degrees_to_radians)

    kAmp = GiraffePos(0.2325, -26 * degrees_to_radians)

    kElevatorHigh = GiraffePos(constants.elevator_max_length, GiraffePos.Special.kCurrentAngle)

    kElevatorLow = GiraffePos(0, GiraffePos.Special.kCurrentAngle)

    kElevatorMid = GiraffePos(constants.elevator_max_length / 2, GiraffePos.Special.kCurrentAngle)


"""
c = drag coefficient
a = projectile area (m^2)
m = projectile mass (kg)
rho_air = air density (kg/m^3)
g = acceleration due to gravity (m/s^2)
v0 = initial velocity of shooter flywheel (m/s) config
delta_x = distance from shooter to target (COULD BE IN ODOMETRY) (m)
y = height of target (COULD BE IN ODOMETRY) (m) const
tol = tolerance of error in distance to target (m)
"""

# Gyro
gyro_id = 29
