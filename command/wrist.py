import utils
import constants
import wpilib
import config
from oi.keymap import Controllers
from toolkit.command import SubsystemCommand, radians
from subsystem import Wrist
from commands2 import SequentialCommandGroup
import math

class ZeroWrist(SubsystemCommand[Wrist]):

    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.zero_wrist()

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.is_at_angle(0)
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.subsystem.wrist_zeroed = True
            utils.LocalLogger.debug("Wrist zeroed")

class SetWrist(SubsystemCommand[Wrist]):

    def __init__(self, subsystem: Wrist, angle: radians):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.angle = angle

    def initialize(self):
        self.subsystem.set_wrist_angle(self.angle)

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.is_at_angle(self.angle)

    def end(self, interrupted:bool):
        self.subsystem.set_wrist_angle(self.subsystem.get_wrist_angle()) #I copied this from the old repo's command, but I'm confused by the logic. pls explain when u see this! thx
        utils.LocalLogger.debug("Wrist position " + str(self.target) + " acheived")

class FeedIn(SubsystemCommand[Wrist]):
    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)

    def initialize(self):
        self.subsystem.feed_in()

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.note_staged

    def end(self):
        utils.LocalLogger.debug("Fed-in")

class FeedOut(SubsystemCommand[Wrist]):
    def __init__(self, subsystem: Wrist, length:float):
        super().__init__(subsystem)

    def initialize(self):
        self.subsystem.feed_out()

    def execute(self):
        pass

    def isFinished(self):
        pass

    def end(self):
        pass