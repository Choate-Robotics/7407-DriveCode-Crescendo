from subsystem import Elevator, Wrist, Intake, Drivetrain
from sensors import FieldOdometry, TrajectoryCalculator

import logging
from command import DriveSwerveCustom, SetElevator, ZeroElevator, ZeroWrist, FeedIn, FeedOut, PassNote, SetWrist, DeployIntake, DeployTenting
import wpilib, config, constants
import commands2
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import WaitUntilCommand, WaitCommand, ParallelCommandGroup, SequentialCommandGroup, InstantCommand, PrintCommand, ParallelDeadlineGroup, RunCommand



class Giraffe(commands2.Command):
    
    def __init__(self, elevator: Elevator, wrist: Wrist, target: config.Giraffe.GiraffePos):
        self.elevator = elevator
        self.wrist = wrist
        self.target = target
        self.finished = False
        self.aiming = False
        self.staging = False
        self.auto_height = False
    
    def finish(self):
        self.finished = True
    
    def initialize(self):
        self.finished = False
        self.aiming = False
        self.staging = False
        self.auto_height = False
        
        
        commands = []
        debug_commands = []
        
        if self.target.height == None or self.target.wrist_angle == None:
            self.finished = True
            return # invalid target
        
        if type(self.target.wrist_angle) == str:
            if self.target.wrist_angle != 'aim' or self.target.wrist_angle != 'stage':
                self.finished = True
                return # invalid string entry, must be 'aim' or 'stage'
        
        if type(self.target.height) == str:
            if self.target.height != 'auto':
                self.finished = True
                return # invalid string entry, must be 'auto'
            
        
        
        debug_commands.append(
            PrintCommand("Running both elevator and wrist normally")
        )
        
        continuous_commands = []
        
        
        # if the target wrist angle is 'aim' then set the wrist angle to the calculated angle, we will pass off the aiming command at the end
        if self.target.wrist_angle == 'aim':
            continuous_commands.append(
                PrintCommand("Aiming wrist")
            )
            self.aiming = True
            self.target.wrist_angle = self.wrist.get_wrist_angle()
            debug_commands.append(
                PrintCommand("Aiming wrist")
            )
        
        # if the target wrist angle is 'stage' then set the wrist angle to the staging angle, we will pass off the staging command at the end
        elif self.target.wrist_angle == 'stage':
            self.staging = True
            self.target.wrist_angle = config.staging_angle
            continuous_commands.append(
                FeedIn(self.wrist)
            )
            debug_commands.append(
                PrintCommand("Staging note to wrist")
            )
        
        if self.target.height == 'auto':
            self.auto_height = True
            self.target.height = self.elevator.get_length()
            continuous_commands.append(
                PrintCommand("Auto height")
            )
            debug_commands.append(
                PrintCommand("Auto height")
            )
        
        
         # if the desired elevator height is greater than 0 while the elevator is locked down
        if self.target.height > config.elevator_stage_max and self.elevator.locked_down or self.target.wrist_angle < config.wrist_stage_max and self.elevator.locked_down:
            self.finished = True
            return # cant perform this action while elevator is locked down
            
        
        
        commands.append(
            ParallelCommandGroup(
                SetElevator(self.elevator, self.target.height),
                SetWrist(self.wrist, self.target.wrist_angle),
                InstantCommand(self.finish),
            )
        )
        
        
        commands += continuous_commands
        
        
        commands2.CommandScheduler.schedule(ParallelCommandGroup(
            SequentialCommandGroup(
                *commands,
            ),
            SequentialCommandGroup(
                *debug_commands
            )
        )
        )
        
    def isFinished(self) -> bool:
        return self.finished
        
    def end(self, interrupted: bool):
        if interrupted:
            self.finished = True
    
class GiraffeLock(commands2.Command):
    
    def __init__(self, elevator: Elevator, wrist: Wrist):
        self.elevator = elevator
        self.wrist = wrist
        self.finished = False
    
    def finish(self):
        self.finished = True
    
    def initialize(self):
        self.finished = False
        
        self.elevator.lock()
        
        def wrist_is_over_limit():
            return self.wrist.get_wrist_angle() > config.wrist_stage_max
        
        def elevator_is_over_limit():
            return self.elevator.get_length() > config.elevator_stage_max
        
        
        commands = []
        debug_commands = []
        
        if not elevator_is_over_limit and not wrist_is_over_limit():
            self.finished = True
            return # already down
            
        if wrist_is_over_limit():
            commands.append(
                SetWrist(self.wrist, config.wrist_stage_max)
            )
            debug_commands.append(
                PrintCommand("Setting wrist to stage limit")
            )
            
        if elevator_is_over_limit():
            commands.append(
                SetElevator(self.elevator, config.elevator_stage_max)
            )
            debug_commands.append(
                PrintCommand("Setting elevator to stage limit")
            )
            
        commands.append(
            InstantCommand(self.finish)
        )
        
        commands2.CommandScheduler.schedule(ParallelCommandGroup(
                *commands,
            SequentialCommandGroup(
                *debug_commands
            )
        )
        )
        
    def isFinished(self) -> bool:
        return self.finished
        
    def end(self, interrupted: bool):
        if interrupted:
            self.finished = True
    
    
class StageNote(SequentialCommandGroup):
    
    def __init__(self, elevator: Elevator, wrist: Wrist, intake: Intake):
        super().__init__(
            Giraffe(elevator, wrist, config.Giraffe.kStage),
            WaitUntilCommand(lambda: intake.detect_note() == False and wrist.note_staged),
            Giraffe(elevator, wrist, config.Giraffe.kIdle),
        )
    
class ShootSpeaker(SequentialCommandGroup):
    
    def __init__(self, drivetrain: Drivetrain, calculations: TrajectoryCalculator, elevator: Elevator, wrist: Wrist, atPose: Pose2d | None = None):
        super().__init__(
            ParallelCommandGroup(
                PrintCommand('Speed up flywheel'),
                Giraffe(elevator, wrist, config.Giraffe.kAim),
                PrintCommand('Aim Drivetrain')
            ),
            WaitUntilCommand(lambda: elevator.ready_to_shoot and wrist.ready_to_shoot and drivetrain.ready_to_shoot),
            PassNote(wrist),
        )
        
class ShootAmp(SequentialCommandGroup):
    
    def __init__(self, drivetrain: Drivetrain, elevator: Elevator, wrist: Wrist):
        super().__init__(
            ParallelCommandGroup(
                PrintCommand('Speed up flywheel'),
                Giraffe(elevator, wrist, config.Giraffe.kAmp),
                PrintCommand('Lock Drivetrain with amp')
            ),
            WaitUntilCommand(lambda: elevator.ready_to_shoot and wrist.ready_to_shoot and drivetrain.ready_to_shoot),
            PassNote(self.wrist),
        )
        
class EnableClimb(SequentialCommandGroup):
    def __init__(self, elevator: Elevator, wrist: Wrist, intake: Intake):
        super().__init__(
            Giraffe(elevator, wrist, config.Giraffe.kClimbReach),
            DeployTenting(intake),
        )