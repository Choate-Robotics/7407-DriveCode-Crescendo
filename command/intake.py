import wpilib
from toolkit.command import SubsystemCommand

import config
from subsystem import Intake

class RunIntake(SubsystemCommand[Intake]):
    # timer = wpilib.Timer()
    timeout: bool

    def initialize(self) -> None:
        self.timeout = False
        # self.timer.reset()
        # self.timer.start()
        self.subsystem.roll_in()
        self.subsystem.intake_running = True

    def execute(self) -> None:
        # self.timeout = True if self.timer.get() >= config.intake_timeout else False
        pass

    def isFinished(self) -> bool:
        # return self.subsystem.detect_note() or self.timeout
        return self.subsystem.detect_note()

    def end(self, interrupted) -> None:
        self.subsystem.rollers_idle_out()
        if not self.timeout or not interrupted:
            self.subsystem.note_in_intake = True
        # self.timer.stop()
        # self.timer.reset()
        # self.subsystem.intake_running = False


class IntakeIdle(SubsystemCommand[Intake]):
    def initialize(self) -> None:
        if self.subsystem.note_in_intake():
            self.subsystem.rollers_idle_out()
        else:
            self.subsystem.rollers_idle_in()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted) -> None:
        pass

class DeployIntake(SubsystemCommand[Intake]):
    def initialize(self) -> None:
        self.subsystem.deploy_roller()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.get_deploy_current() > config.intake_deploy_current_limit
    
    def end(self, interrupted) -> None:
        self.subsystem.deploy_motor.set_raw_output(0)

class DeployTenting(SubsystemCommand[Intake]):
    def initialize(self) -> None:
        self.subsystem.deploy_roller()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.get_deploy_current() > config.tenting_deploy_current_limit
    
    def end(self, interrupted) -> None:
        self.subsystem.deploy_motor.set_raw_output(0)