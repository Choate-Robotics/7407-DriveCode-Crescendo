from wpilib import TimedRobot
import time
# from phoenix6 import StatusFrameEnhanced
# from toolkit.motors import TalonFX
# from toolkit.motors.ctre_motors import _Talon


# def optimize_normal_talon(t: TalonFX | _Talon):
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 0)
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 0)
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 0, 20)


# def optimize_normal_talon_no_sensor(t: TalonFX | _Talon):
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 0)
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 0)
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 0)


# def optimize_leader_talon(t: TalonFX | _Talon):
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, 0)
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 0)
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 0)


# def optimize_leader_talon_no_sensor(t: TalonFX | _Talon):
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, 0)
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 0)
#     t._motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 0)

def CAN_delay(seconds: float):
    '''
    Delays the code by the given amount of seconds, but only if the robot is not in simulation mode
    
    Useful for the CAN bus, which may need to catch up during initialization
    '''
    time.sleep(seconds) if not TimedRobot.isSimulation() else None