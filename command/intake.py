import wpilib
from toolkit.command import SubsystemCommand
from wpilib import Timer
import config
from subsystem import Intake

class RunIntake(SubsystemCommand[Intake]):

    def initialize(self) -> None:
        self.subsystem.roll_in()
        self.subsystem.intake_running = True
        self.note_detected = False

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        self.note_detected = self.subsystem.detect_note()
        return self.note_detected

    def end(self, interrupted) -> None:
        self.subsystem.stop_inner()
        if not interrupted and self.note_detected:
            self.subsystem.note_in_intake = True
            self.subsystem.rollers_idle_out()
        self.subsystem.intake_running = False
        
class EjectIntake(SubsystemCommand[Intake]):
    def initialize(self) -> None:
        self.subsystem.roll_out()
        self.subsystem.intake_running = True

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop_inner()
        self.subsystem.intake_running = False


class IntakeIdle(SubsystemCommand[Intake]):
    def initialize(self) -> None:
        if self.subsystem.note_in_intake:
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
        self.timer = Timer()
        self.timer.start()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return (
            self.subsystem.get_deploy_current() > config.intake_deploy_current_limit
            and
            self.timer.get() > config.deploy_intake_timeout
        )
    
    def end(self, interrupted) -> None:
        self.subsystem.deploy_motor.set_raw_output(0)
        if not interrupted:
            self.subsystem.intake_deployed = True

class DeployTenting(SubsystemCommand[Intake]):
    def initialize(self) -> None:
        self.subsystem.deploy_tenting()
        self.timer = Timer()
        self.timer.start()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return (
            self.subsystem.get_deploy_current() > config.tenting_deploy_current_limit
            and
            self.timer.get() > config.deploy_tenting_timeout)
    
    def end(self, interrupted) -> None:
        self.subsystem.deploy_motor.set_raw_output(0)