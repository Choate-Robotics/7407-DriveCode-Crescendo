from __future__ import annotations
from toolkit.motors import SparkMax
from rev import CANSparkMax



class RevPeriodicFrames:
    
    @property
    def k0():
        '''
        Applied Output, Faults, Sticky Faults, isFollower
        
        Default Period: 10ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus0
    
    @property
    def k1():
        '''
        Motor Velocity, Motor Current, Motor Voltage, Motor Temperature
        
        Default Period: 20ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus1
    
    @property
    def k2():
        '''
        Motor Position
        
        Default Period: 20ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus2
    
    @property
    def k3():
        '''
        Analog Sensor Voltage, Analog Sensor Position, Analog Sensor Velocity
        
        Default Period: 50ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus3
    
    @property
    def k4():
        '''
        Alternate Encoder Position, Alternate Encoder Velocity
        
        Default Period: 20ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus4
    
    @property
    def k5():
        '''
        Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Angle
        
        Default Period: 200ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus5
    
    @property
    def k6():
        '''
        Duty Cycle Absolute Encoder Velocity, Duty Cycle Absolute Encoder Frequency
        
        Default Period: 200ms
        '''
        return CANSparkMax.PeriodicFrame.kStatus6
    
    
max_period_rev = 32767

optimized_basic_period_rev = 15

def maximize_frame_period_rev(s: SparkMax, frame: RevPeriodicFrames):
    s.motor.setPeriodicFramePeriod(frame, max_period_rev)


def optimize_normal_sparkmax(s: SparkMax):
    maximize_frame_period_rev(s, RevPeriodicFrames.k3)
    maximize_frame_period_rev(s, RevPeriodicFrames.k4)
    maximize_frame_period_rev(s, RevPeriodicFrames.k5)
    maximize_frame_period_rev(s, RevPeriodicFrames.k6)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0, 10)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k1, optimized_basic_period_rev)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k2, optimized_basic_period_rev)
    
def optimize_sparkmax_analog_sensor(s: SparkMax):
    maximize_frame_period_rev(s, RevPeriodicFrames.k4)
    maximize_frame_period_rev(s, RevPeriodicFrames.k5)
    maximize_frame_period_rev(s, RevPeriodicFrames.k6)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0, 10)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k1, optimized_basic_period_rev)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k2, optimized_basic_period_rev)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k3, 20)
    

def optimize_sparkmax_absolute_encoder(s: SparkMax):
    maximize_frame_period_rev(s, RevPeriodicFrames.k3)
    maximize_frame_period_rev(s, RevPeriodicFrames.k4)
    maximize_frame_period_rev(s, RevPeriodicFrames.k6)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0, 10)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k1, optimized_basic_period_rev)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k2, optimized_basic_period_rev)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k5, 50)
    
def optimize_sparkmax_absolute_encoder_all(s: SparkMax):
    maximize_frame_period_rev(s, RevPeriodicFrames.k3)
    maximize_frame_period_rev(s, RevPeriodicFrames.k4)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0, 10)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k1, optimized_basic_period_rev)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k2, optimized_basic_period_rev)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k5, 100)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k6, 100)
    
def optimize_sparkmax_no_position(s: SparkMax):
    maximize_frame_period_rev(s, RevPeriodicFrames.k2)
    maximize_frame_period_rev(s, RevPeriodicFrames.k3)
    maximize_frame_period_rev(s, RevPeriodicFrames.k4)
    maximize_frame_period_rev(s, RevPeriodicFrames.k5)
    maximize_frame_period_rev(s, RevPeriodicFrames.k6)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k0, 10)
    s.motor.setPeriodicFramePeriod(RevPeriodicFrames.k1, optimized_basic_period_rev)
    