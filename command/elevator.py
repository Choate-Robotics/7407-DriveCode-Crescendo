import utils
import constants

from toolkit.command import SubsystemCommand
from subsystem import Elevator

class ZeroElevator(SubsystemCommand[Elevator]):
        
        def __init__(self, subsystem: Elevator):
            super().__init__(subsystem)
    
        def initialize(self):
            self.subsystem.zero()
    
        def execute(self):
            pass
    
        def isFinished(self):
            return self.subsystem.zeroed
        
        def end(self, interrupted: bool):
            if interrupted:
                # utils.LocalLogger.debug("Ending elevator", "ZeroElevator")
                self.subsystem.stop()
            else:
                ...
                # utils.LocalLogger.debug("Elevator zeroed: " + str(self.subsystem.zeroed), "ZeroElevator")


class SetElevator(SubsystemCommand[Elevator]):
    
    def __init__(self, subsystem: Elevator, length: float):
        super().__init__(subsystem)
        self.length: float = length
        self.elevator_moving: bool = False

    def initialize(self):

        self.length = self.subsystem.limit_length(self.length)

        self.subsystem.set_length(self.length)
        self.elevator_moving = True

    def execute(self):
        pass

    def isFinished(self):
        # Rounding to make sure it's not too precise (will cause err)
        return round(self.subsystem.get_length(), 2) == round(self.length, 2)
    
    def end(self, interrupted: bool):
        if interrupted:
            # utils.LocalLogger.debug("Ending elevator", "SetElevator")
            self.subsystem.stop()
        else:
            ...
            # utils.LocalLogger.debug("Elevator length: " + str(self.subsystem.get_length()), "SetElevator")

        self.elevator_moving = False
        
    