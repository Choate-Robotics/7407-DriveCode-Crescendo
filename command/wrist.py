import utils
import constants
import wpilib
import config
from oi.keymap import Controllers
from toolkit.command import SubsystemCommand
from subsystem import Wrist
from commands2 import SequentialCommandGroup
from units.SI import radians
import math
from sensors import TrajectoryCalculator


class ZeroWrist(SubsystemCommand[Wrist]):
    """
    Zeroes wrist
    """
    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.zero_wrist()

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.wrist_zeroed

    def end(self, interrupted: bool):
        if not interrupted:
            self.subsystem.wrist_zeroed = True
            utils.LocalLogger.debug("Wrist zeroed")
        else:
            utils.LocalLogger.debug("Wrist zeroing interrupted")


class SetWrist(SubsystemCommand[Wrist]):
    """
    Set wrist to given angle.
    If interrupted, stops wrist where it is.
    param angle: angle to set wrist to in radians
    """
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

    def end(self, interrupted: bool):
        if interrupted:
            wrist_angle = self.subsystem.get_wrist_angle()
            self.subsystem.set_wrist_angle(wrist_angle)  #stopping motor where it is
            # utils.LocalLogger.debug("Interrupted, Wrist position " + str(wrist_angle))
        # else:
        #     utils.LocalLogger.debug("Wrist position " + str(self.angle) + " acheived")


class AimWrist(SubsystemCommand[Wrist]):
    """
    Aims wrist to angle according to shooter calculations
    """
    def __init__(self, subsystem: Wrist, traj_calc: TrajectoryCalculator):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.traj_calc = traj_calc

    def initialize(self):
        pass

    def execute(self):
        self.subsystem.set_wrist_angle(self.traj_calc.get_theta())

        if self.subsystem.is_at_angle(self.traj_calc.get_theta()):
            self.subsystem.ready_to_shoot = True
        else:
            self.subsystem.ready_to_shoot = False

    def isFinished(self):
        return False

    def end(self, interrupted: bool):
        self.subsystem.ready_to_shoot = False
        if interrupted:
            wrist_angle = self.subsystem.get_wrist_angle()
            self.subsystem.set_wrist_angle(wrist_angle)  #stopping motor where it is
            # utils.LocalLogger.debug("Interrupted, Wrist position " + str(wrist_angle))
        # else:
        #     utils.LocalLogger.debug("Wrist position " + str(self.angle) + " acheived")


class FeedIn(SubsystemCommand[Wrist]):
    """
    Feed note into back of feeder
    """
    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.feed_in()

    def execute(self):
        voltage = config.feeder_voltage * (
        (config.feeder_sensor_threshold - self.subsystem.distance_sensor.getVoltage()))

        self.subsystem.set_feed_voltage(voltage)

    def isFinished(self):
        return self.subsystem.note_detected()
        # return True

    def end(self, interrupted: bool):
        self.subsystem.stop_feed()
        if not interrupted:
            self.subsystem.note_staged = True
            # utils.LocalLogger.debug("Fed-in")


class FeedOut(SubsystemCommand[Wrist]):
    """
    Feed note out back of feeder
    """
    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.feed_out()

    def execute(self):
        pass

    def isFinished(self):
        return not self.subsystem.note_staged

    def end(self, interrupted: bool):
        self.subsystem.stop_feed()
        # if interrupted:
        #     # utils.LocalLogger.debug("Feed out interrupted")
        # else:
        #     # utils.LocalLogger.debug("Fed-out")


class PassNote(SubsystemCommand[Wrist]):
    """
    Pass note into flywheels
    """
    def initialize(self):
        self.subsystem.feed_note()

    def execute(self):
        pass

    def isFinished(self):
        return not self.subsystem.note_detected()
        # need to include beam break

    def end(self, interrupted: bool):
        self.subsystem.stop_feed()
        # if interrupted:
        #     ...
        #     # utils.LocalLogger.debug("Note transfer interrupted")
        # else:
        #     # utils.LocalLogger.debug("Note transferred")
        # self.subsystem.note_staged = False
