import pytest
from toolkit.motors.rev_motors import SparkMax
from toolkit.subsystem import Subsystem as Subsystem_C
from rev import CANSparkMax

# to test a subsystem, you need to create a mechanism

# this will override the subsystem's motors and devices with mock versions of them

# this will allow you to test the subsystem without having to worry about the motors and devices, and streamline the process

# if you need to add more motors or devices, you can create a child class of the device and override the methods you need to test

# examples of this are the MockSparkMotor and MockTalonFX classes below

# you can also remove motors and devices by using the remove_parent_variables method in the Mechanism_C class


def mechanism(selected_subsystem: Subsystem_C):
    
    class MockSparkMotor(SparkMax):
        
        class PIDController:
                
                def __init__(self, motor: SparkMax):
                    self._motor = motor
                    
                def setReference(self, value, control_type):
                    if control_type == CANSparkMax.ControlType.kPosition:
                        self._motor._position = value
                    elif control_type == CANSparkMax.ControlType.kVelocity:
                        self._motor._velocity = value
                    elif control_type == CANSparkMax.ControlType.kVoltage:
                        self._motor._output = value
                        
        class Motor:
            
            def __init__(self, motor: SparkMax):
                self._motor = motor
                
            def set(self, value):
                self._motor._output = value
                
            def getAppliedOutput(self):
                return self._motor._output
        
        def __init__(self, motor: SparkMax):
            super().__init__(motor._can_id, motor._inverted, motor._brushless, motor._config)
            self._output = 0
            self._velocity = 0
            self._position = 0
            self.pid_controller = self.PIDController(self)
            self.motor = self.Motor(self)
            
        def set_target_velocity(self, vel):
            self._velocity = vel
            
        def get_sensor_velocity(self):
            return self._velocity
        
        def get_target_velocity(self):
            return self._velocity
        
        def set_target_position(self, target):
            self._position = target
            
        def get_sensor_position(self):
            return self._position
        
    class Mechanism_C(selected_subsystem):
        
        def __init__(self):
            super().__init__()
            
        def init(self): 
            
            self.replace_parent_variables(SparkMax, MockSparkMotor)
            # replace any motors or devices here

            super().init()
            
        def get_parent_variables_by_type(self, type):
            '''
            Returns a list of all the variable names of the parent class that are of the specified type
            '''
            return  [attr for attr in dir(selected_subsystem) if not callable(getattr(selected_subsystem, attr)) and not attr.startswith("__") and isinstance(getattr(selected_subsystem, attr), type)]
            
        def replace_parent_variables(self, type, new_type):
            '''
            Replaces all the variables of the parent class that are of the specified type with the new type
            '''
            for attr in self.get_parent_variables_by_type(type):
                super().__setattr__(attr, new_type(getattr(selected_subsystem, attr)))
                
        def set_parent_variables(self, type, value):
            '''
            Sets all the variables of the parent class that are of the specified type to the specified value
            
            NOTE: This is setting the value to a primitive type, so it will not work for objects
            '''
            for attr in self.get_parent_variables_by_type(type):
                super().__setattr__(attr, value)
                
        def remove_parent_variables(self, type):
            '''
            Removes all the variables of the parent class that are of the specified type
            '''
            for attr in self.get_parent_variables_by_type(type):
                delattr(self, attr)
            
        
        def deconstruct(self):
            self.remove_parent_variables(SparkMax)
            # remove any motors or devices here
    
    return Mechanism_C()
        
        
