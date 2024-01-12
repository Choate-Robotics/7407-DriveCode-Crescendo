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

class Type():
        
    def KStatic(r, g, b):
        return {
            'type': 1,
            'color': {
                'r': r,
                'g': g,
                'b': b
            }
        }
    
    def KRainbow():
        return {
            'type': 2
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
    
    def KBlink(r,g,b):
        return {
            'type': 4,
            'color': {
                'r': r,
                'g': g,
                'b': b
            }
        }
        
    def KLadder(typeA,typeB,percent,speed):
        return {
            'type': 5,
            'percent': percent, # 0-1
            'typeA': typeA,
            'typeB': typeB,
            'speed': speed
        }
    
    
    
class Team:
        
    red = 0
    blue = 1
    
    
class LimelightPipeline:
    
    feducial = 0.0
    neural = 1.0
    retroreflective = 2.0

limelight_led_mode: int = {
    'pipeline_default': 0,
    'force_off': 1,
    'force_blink': 2,
    'force_on': 3
    
}

active_team: Team = Team.blue