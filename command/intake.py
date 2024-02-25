import wpilib
from toolkit.command import SubsystemCommand
from wpilib import Timer
import config
from subsystem import Intake


class RunIntake(SubsystemCommand[Intake]):
    """
    Runs intake in until note detected
    """
    def initialize(self) -> None:
        self.subsystem.roll_in()
        self.subsystem.intake_running = True
        self.note_detected = False

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.note_in_intake

    def end(self, interrupted) -> None:
        self.subsystem.stop_inner()
        if self.subsystem.note_in_intake:
            self.subsystem.rollers_idle_out()
        self.subsystem.intake_running = False


class PassIntakeNote(SubsystemCommand[Intake]):
    """
    Pass note from intake to feeder
    """
    def initialize(self) -> None:
        self.subsystem.roll_inner_in()
        self.subsystem.intake_running = True
        self.note_gone = False

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return not self.subsystem.note_in_intake

    def end(self, interrupted) -> None:
        self.subsystem.stop_inner()
        # if not interrupted and self.note_gone:
        # self.subsystem.note_in_intake = False
        # self.subsystem.rollers_idle_in()
        self.subsystem.intake_running = False
        self.subsystem.note_in_intake = False


class EjectIntake(SubsystemCommand[Intake]):
    """
    Eject note out of intake
    """
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
        self.subsystem.note_in_intake = False


class IntakeIdle(SubsystemCommand[Intake]):
    """
    Command to run while not actively intaking.
    Intake runs in if we don't have a note, runs in if we do
    """
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
    """
    Command to run at start of match.
    Deploys outer intake
    """
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
    """
    Deploy tenting mechanism for climb
    """
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


class UnDeployTenting(SubsystemCommand[Intake]):
    """
    Undeploys tenting mech incase we don't want to climb
    """
    def initialize(self) -> None:
        self.subsystem.undeploy_tenting()
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
