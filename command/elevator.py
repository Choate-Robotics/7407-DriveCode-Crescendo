import utils
import constants, config

from toolkit.command import SubsystemCommand
from subsystem import Elevator

class ZeroElevator(SubsystemCommand[Elevator]):
        """
        Zeroes elevator
        """
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
                ...
                # utils.LocalLogger.debug("Ending elevator", "ZeroElevator")
                # self.subsystem.stop()
            else:
                ...
                # utils.LocalLogger.debug("Elevator zeroed: " + str(self.subsystem.zeroed), "ZeroElevator")


class SetElevator(SubsystemCommand[Elevator]):
    """
    Set elevator to specified length.
    param length: length to set elevator to (float)
    """
    def __init__(self, subsystem: Elevator, length: float):
        super().__init__(subsystem)
        self.length: float = length


    def initialize(self):

        self.length = self.subsystem.limit_length(self.length)
        
        self.subsystem.set_length(self.length, 0)
        self.subsystem.elevator_moving = True

    def execute(self):
        pass

    def isFinished(self):
        # Rounding to make sure it's not too precise (will cause err)
        return round(self.subsystem.get_length(), 2) == round(self.length, 2)
    
    def end(self, interrupted: bool):
        if interrupted:
            ...
            # utils.LocalLogger.debug("Ending elevator", "SetElevator")
            # self.subsystem.stop()
        else:
            ...
            # utils.LocalLogger.debug("Elevator length: " + str(self.subsystem.get_length()), "SetElevator")

        self.subsystem.elevator_moving = False
        
    
    
class SetElevatorClimbDown(SubsystemCommand[Elevator]):
    """
    Climb down with feed forward
    """
    def __init__(self, subsystem: Elevator):
        super().__init__(subsystem)

    def initialize(self):

        self.subsystem.set_elevator_climb_down()
        self.subsystem.elevator_moving = True

    def execute(self):
        pass

    def isFinished(self):
        # Rounding to make sure it's not too precise (will cause err)
        return round(self.subsystem.get_length(), 2) <= 0 and self.subsystem.get_elevator_current() > config.elevator_climb_current_limit
    
    def end(self, interrupted: bool):
        self.subsystem.stop()
        if interrupted:
            ...
            # utils.LocalLogger.debug("Ending elevator", "SetElevator")
            # self.subsystem.stop()
        else:
            ...
            # utils.LocalLogger.debug("Elevator length: " + str(self.subsystem.get_length()), "SetElevator")

        self.subsystem.elevator_moving = False
        