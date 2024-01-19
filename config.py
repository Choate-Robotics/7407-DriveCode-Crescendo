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

#intake
inner_intake_id = 0 #placeholder
outer_intake_front_id = 1 #placeholder
outer_intake_back_id = 2 #placeholder
intake_beam_break_channel = 1 #placeholder

intake_inner_speed = 0.25 #placeholder
intake_outer_speed = 0.5 #placeholder
intake_outer_idle_speed = .25 #placeholder