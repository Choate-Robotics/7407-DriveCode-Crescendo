import math

import wpilib  # noqa
from commands2 import SequentialCommandGroup  # noqa

import config
import constants  # noqa
import utils  # noqa
from oi.keymap import Controllers  # noqa
from sensors import TrajectoryCalculator
from subsystem import Wrist
from toolkit.command import SubsystemCommand
from units.SI import radians


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
            # utils.LocalLogger.debug("Wrist zeroed")
        else:
            ...
            # utils.LocalLogger.debug("Wrist zeroing interrupted")


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
        # self.subsystem.zero_wrist()
        self.subsystem.set_wrist_angle(self.angle)
        self.subsystem.wrist_moving = True

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.is_at_angle(self.angle)

    def end(self, interrupted: bool):
        if interrupted:
            wrist_angle = self.subsystem.get_wrist_angle()  # noqa
            # self.subsystem.set_wrist_angle(wrist_angle)  #stopping motor where it is
            # utils.LocalLogger.debug("Interrupted, Wrist position " + str(wrist_angle))
        self.subsystem.wrist_moving = False
        #     utils.LocalLogger.debug("Wrist position " + str(self.angle) + " acheived")


class SetWristIdle(SubsystemCommand[Wrist]):
    """
    Set wrist to given angle.
    If interrupted, stops wrist where it is.
    param angle: angle to set wrist to in radians
    """

    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        # self.subsystem.zero_wrist()
        self.subsystem.set_wrist_angle(config.staging_angle)
        self.subsystem.wrist_moving = True

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.is_at_angle(config.staging_angle)

    def end(self, interrupted: bool):
        if interrupted:
            wrist_angle = self.subsystem.get_wrist_angle()  # noqa
            # self.subsystem.set_wrist_angle(wrist_angle)  #stopping motor where it is
            # utils.LocalLogger.debug("Interrupted, Wrist position " + str(wrist_angle))
        self.subsystem.wrist_moving = False
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
        self.subsystem.wrist_moving = True

    def execute(self):
        # angle = self.traj_calc.get_theta()

        self.subsystem.set_wrist_angle(self.traj_calc.get_theta())

        if self.subsystem.is_at_angle(self.traj_calc.get_theta(), math.radians(1)):
            self.subsystem.ready_to_shoot = True
        else:
            self.subsystem.ready_to_shoot = False

    def isFinished(self):
        return False

    def end(self, interrupted: bool):
        self.subsystem.ready_to_shoot = False
        if interrupted:
            wrist_angle = self.subsystem.get_wrist_angle()  # noqa
            # self.subsystem.set_wrist_angle(wrist_angle)
            # stopping motor where it is
            # utils.LocalLogger.debug("Interrupted, Wrist position " + str(wrist_angle))
        self.subsystem.wrist_moving = False
        #     utils.LocalLogger.debug("Wrist position " + str(self.angle) + " acheived")


class FeedIn(SubsystemCommand[Wrist]):
    """
    Feed note into back of feeder.
    Start by going fast, until first beam break sees note, then slow down.
    Stop when second beam break sees note.
    """

    def __init__(self, subsystem: Wrist):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.first_note_detected: bool = False

    def initialize(self):
        if not self.subsystem.detect_note_second():
            self.subsystem.feed_in()

    def execute(self):
        if self.subsystem.detect_note_first():
            self.first_note_detected = True

        voltage = (
            config.feeder_voltage_crawl
            if self.first_note_detected
            else config.feeder_voltage_feed
        )

        if not self.subsystem.detect_note_second():
            self.subsystem.set_feed_voltage(voltage)

    def isFinished(self):
        return self.subsystem.detect_note_second()
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
        # return self.subsystem.detect_note_first()
        return True

    def end(self, interrupted: bool):
        ...
        # self.subsystem.stop_feed()
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

    def end(self, interrupted: bool):
        self.subsystem.stop_feed()
        # if interrupted:
        #     ...
        #     # utils.LocalLogger.debug("Note transfer interrupted")
        # else:
        #     # utils.LocalLogger.debug("Note transferred")
        # self.subsystem.note_staged = False
