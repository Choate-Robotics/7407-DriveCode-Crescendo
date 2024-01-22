import wpilib
from toolkit.command import SubsystemCommand

import config
from subsystem import Intake

class RunIntake(SubsystemCommand[Intake]):
    timer = wpilib.Timer()
    timeout: bool

    def initialize(self) -> None:
        self.timeout = False
        self.timer.reset()
        self.timer.start()
        self.subsystem.roll_in()

    def execute(self) -> None:
        self.timeout = self.timer.get() >= config.intake_timeout

    def isFinished(self) -> bool:
        return self.subsystem.detect_note() or self.timeout

    def end(self) -> None:
        self.subsystem.rollers_idle_out()
        if not self.timeout:
            self.subsystem.note_in_intake = True
        self.timer.stop()
        self.timer.reset()