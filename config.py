from enum import Enum
from dataclasses import dataclass

from units.SI import (
    inches_to_meters,
    meters,
    meters_per_second,
    meters_per_second_squared,
    radians,
)

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

# LEDS
def KRainbow():
    return {
        'type': 2
    }


class Type:

    def KStatic(r, g, b):
        return {
            'type': 1,
            'color': {
                'r': r,
                'g': g,
                'b': b
            }
        }

    def KTrack(r1, g1, b1, r2, g2, b2):
        return {
            'type': 3,
            'color': {
                'r1': r1,
                'g1': g1,
                'b1': b1,
                'r2': r2,
                'g2': g2,
                'b2': b2
            }
        }

    def KBlink(r, g, b):
        return {
            'type': 4,
            'color': {
                'r': r,
                'g': g,
                'b': b
            }
        }

    def KLadder(typeA, typeB, percent, speed):
        return {
            'type': 5,
            'percent': percent,  # 0-1
            'typeA': typeA,
            'typeB': typeB,
            'speed': speed
        }


# TEAM
class Team(Enum):
    RED = 0
    BLUE = 1


active_team: Team = Team.BLUE


# LIMELIGHT
class LimelightPipeline:
    feducial = 0.0
    neural = 0.0
    retroreflective = 0.0


limelight_led_mode = {
    'pipline_default': 0,
    'force_off': 1,
    'force_blink': 2,
    'force_on': 3
}

# DRIVETRAIN
front_left_move_id = 0
front_left_turn_id = 1
front_left_encoder_port = 0
front_left_encoder_zeroed_pos = 0

front_right_move_id = 2
front_right_turn_id = 3
front_right_encoder_port = 1
front_right_encoder_zeroed_pos = 0

back_left_move_id = 4
back_left_turn_id = 5
back_left_encoder_port = 2
back_left_encoder_zeroed_pos = 0

back_right_move_id = 6
back_right_turn_id = 8
back_right_encoder_port = 3
back_right_encoder_zeroed_pos = 0

driver_centric: bool = True
drivetrain_reversed: bool = False

# Gyro
gyro_id = 0
