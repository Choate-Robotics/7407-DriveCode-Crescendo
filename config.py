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

elevator_can_id: int = 1 # TODO: PLACEHOLDER
elevator_ramp_rate: float = 1.0 # TODO: PLACEHOLDER
elevator_max_rotation: float = 1.0 # TODO: PLACEHOLDER
elevator_auto_position: float = 1.0 # TODO: PLACEHOLDER