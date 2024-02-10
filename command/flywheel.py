import utils
import constants

from toolkit.command import SubsystemCommand
from subsystem import Flywheel
from units.SI import meters_per_second

class SetFlywheelLinearVelocity(SubsystemCommand[Flywheel]):
        
        def __init__(self, subsystem: Flywheel, velocity: meters_per_second):
            super().__init__(subsystem)
            self.subsystem = subsystem
            self.velocity = velocity
    
        def initialize(self):
            self.subsystem.set_velocity_linear(self.velocity)
    
        def execute(self):
            pass
    
        def isFinished(self):
            return False
        
        def end(self, interrupted: bool):
            pass
                
class SetFlywheelVelocityIndependent(SubsystemCommand[Flywheel]):
        
        def __init__(self, subsystem: Flywheel, velocity: tuple[meters_per_second, meters_per_second]):
            super().__init__(subsystem)
            self.subsystem = subsystem
            self.velocity = velocity
    
        def initialize(self):
            self.subsystem.set_velocity_linear(self.velocity[0], 1)
            self.subsystem.set_velocity_linear(self.velocity[1], 2)
    
        def execute(self):
            pass
    
        def isFinished(self):
            return False
        
        def end(self, interrupted: bool):
            pass
