from __future__ import annotations
from rev import CANSparkMax
from wpilib import TimedRobot


class RevPeriodicFrames:
    

    def k0():
        '''
        Applied Output, Faults, Sticky Faults, isFollower
        
        Default Period: 10ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus0
    

    def k1():
        '''
        Motor Velocity, Motor Current, Motor Voltage, Motor Temperature
        
        Default Period: 20ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus1
    
    def k2():
        '''
        Motor Position
        
        Default Period: 20ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus2
    
    def k3():
        '''
        Analog Sensor Voltage, Analog Sensor Position, Analog Sensor Velocity
        
        Default Period: 50ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus3

    def k4():
        '''
        Alternate Encoder Position, Alternate Encoder Velocity
        
        Default Period: 20ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus4
    
    def k5():
        '''
        Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Angle
        
        Default Period: 200ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus5
    
    def k6():
        '''
        Duty Cycle Absolute Encoder Velocity, Duty Cycle Absolute Encoder Frequency
        
        Default Period: 200ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus6
    
    
max_period_rev = 32767

optimized_basic_period_rev = 15

def maximize_frame_period_rev(s: CANSparkMax, frame: RevPeriodicFrames):
    s.setPeriodicFramePeriod(frame, max_period_rev)

def check_simulation():
    return TimedRobot.isSimulation()


def optimize_normal_sparkmax(s: CANSparkMax):
    if check_simulation():
        return
    maximize_frame_period_rev(s, RevPeriodicFrames.k3())
    maximize_frame_period_rev(s, RevPeriodicFrames.k4())
    maximize_frame_period_rev(s, RevPeriodicFrames.k5())
    maximize_frame_period_rev(s, RevPeriodicFrames.k6())
    s.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k1(), optimized_basic_period_rev)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k2(), optimized_basic_period_rev)
    
def optimize_sparkmax_analog_sensor(s: CANSparkMax):
    if check_simulation():
        return
    maximize_frame_period_rev(s, RevPeriodicFrames.k4())
    maximize_frame_period_rev(s, RevPeriodicFrames.k5())
    maximize_frame_period_rev(s, RevPeriodicFrames.k6())
    s.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k1(), optimized_basic_period_rev)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k2(), optimized_basic_period_rev)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k3(), 20)
    

def optimize_sparkmax_absolute_encoder(s: CANSparkMax):
    if check_simulation():
        return
    maximize_frame_period_rev(s, RevPeriodicFrames.k3())
    maximize_frame_period_rev(s, RevPeriodicFrames.k4())
    maximize_frame_period_rev(s, RevPeriodicFrames.k6())
    s.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k1(), optimized_basic_period_rev)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k2(), optimized_basic_period_rev)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k5(), 50)
    
def optimize_sparkmax_absolute_encoder_all(s: CANSparkMax):
    if check_simulation():
        return
    maximize_frame_period_rev(s, RevPeriodicFrames.k3())
    maximize_frame_period_rev(s, RevPeriodicFrames.k4())
    s.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k1(), optimized_basic_period_rev)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k2(), optimized_basic_period_rev)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k5(), 100)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k6(), 100)
    
def optimize_sparkmax_no_position(s: CANSparkMax):
    if check_simulation():
        return
    maximize_frame_period_rev(s, RevPeriodicFrames.k2())
    maximize_frame_period_rev(s, RevPeriodicFrames.k3())
    maximize_frame_period_rev(s, RevPeriodicFrames.k4())
    maximize_frame_period_rev(s, RevPeriodicFrames.k5())
    maximize_frame_period_rev(s, RevPeriodicFrames.k6())
    s.setPeriodicFramePeriod(RevPeriodicFrames.k0(), 10)
    s.setPeriodicFramePeriod(RevPeriodicFrames.k1(), optimized_basic_period_rev)
    