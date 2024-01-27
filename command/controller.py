from subsystem import Elevator, Wrist, Intake, Drivetrain
from sensors import FieldOdometry

import logging
from command import DriveSwerveCustom, SetElevator, ZeroElevator
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
    
    def finish(self):
        self.finished = True
    
    def initialize(self):
        self.finished = False
        self.aiming = False
        self.staging = False
        
        def wrist_is_over_limit():
            return self.wrist.get_wrist_angle() > config.wrist_rotation_limit
        
        def wrist_target_over_limit():
            return self.target.wrist_angle > config.wrist_rotation_limit
        
        def elevator_is_over_limit():
            return self.elevator.get_length() > config.elevator_wrist_limit
        
        def elevator_target_over_limit():
            return self.target.height > config.elevator_wrist_limit
        
        commands = []
        debug_commands = []
        
        if self.target.height == None or self.target.wrist_angle == None:
            return # invalid target
        
        if type(self.target.wrist_angle) == str:
            if self.target.wrist_angle != 'aim' or self.target.wrist_angle != 'stage':
                return # invalid string entry, must be 'aim' or 'stage'
        
        
        # if the target wrist angle is 'aim' then set the wrist angle to the calculated angle, we will pass off the aiming command at the end
        if self.target.wrist_angle == 'aim':
            self.aiming = True
            self.target.wrist_angle = 0 # initial wrist angle
        # if the target wrist angle is 'stage' then set the wrist angle to the staging angle, we will pass off the aiming command at the end
        elif self.target.wrist_angle == 'stage':
            self.staging = True
            self.target.wrist_angle = config.staging_angle
        
        # if the desired elevator height is lower than the wrist limit threshold and the desired wrist angle is over the threshold
        if not elevator_target_over_limit() and wrist_target_over_limit():
            return # exceeds limits of wrist and elevator constraints
        
        # if the desired elevator height is greater than 0 while the elevator is locked down
        if self.target.height > 0.1 and self.elevator.locked_down or self.target.wrist_angle < 0 and self.elevator.locked_down:
            return # cant perform this action while elevator is locked down
            
        
        # if the desired elevator height is lower than the wrist limit threshold and the wrist is over the threshold
        if not elevator_target_over_limit() and wrist_is_over_limit():
            # move wrist out of the way
            commands.append(
                RunCommand(lambda: self.wrist.set_wrist_angle(self.target.wrist_angle)))\
                    .until(lambda: self.wrist.get_wrist_angle() > config.wrist_rotation_limit)\
                    .andThen(ParallelCommandGroup(
                        SetElevator(self.elevator, self.target.height),
                        InstantCommand(lambda: self.wrist.set_wrist_angle(self.target.wrist_angle)),
                    )
            )
                    
            debug_commands.append(
                PrintCommand("Wrist over limit, moving wrist out of the way before running elevator")
            )
        elif wrist_target_over_limit() and not elevator_is_over_limit():
            # move elevator up before running wrist
            commands.append(
                SetElevator(self.elevator, self.target.height)\
                    .until(elevator_is_over_limit)\
                    .andThen(ParallelCommandGroup(
                        SetElevator(self.elevator, self.target.height),
                        InstantCommand(lambda: self.wrist.set_wrist_angle(self.target.wrist_angle)),
                    )
                )
            )
            
            debug_commands.append(
                PrintCommand("Elevator under limit, moving elevator up before running wrist")
            )
            
        else:
            # run both at the same time
            commands.append(
                ParallelCommandGroup(
                    SetElevator(self.elevator, self.target.height),
                )
            )
            debug_commands.append(
                PrintCommand("Running both elevator and wrist normally")
            )
        
        commands2.CommandScheduler.schedule(ParallelCommandGroup(
            SequentialCommandGroup(
                *commands,
                InstantCommand(self.finish),
                PrintCommand("Start aiming here with command") if self.aiming else PrintCommand("Not aiming"),
                PrintCommand("Start staging here with command") if self.staging else PrintCommand("Not staging"),
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
    
    
    def initialize(self):
        
        def wrist_is_over_limit():
            return self.wrist.get_wrist_angle() > config.wrist_rotation_limit
        
        def wrist_under_lock_limit():
            return self.wrist.get_wrist_angle() > -0.1
        
        def elevator_is_over_limit():
            return self.elevator.get_length() > config.elevator_wrist_limit
        
        
        commands = []
        debug_commands = []
            
        # if the elevator is over the limit and the wrist is over the limit
        if elevator_is_over_limit() and wrist_is_over_limit():
            # move wrist out of the way
            commands.append(
                RunCommand(lambda: self.wrist.set_wrist_angle(0)))\
                    .until(lambda: self.wrist.get_wrist_angle() > config.wrist_rotation_limit)\
                    .andThen(ParallelCommandGroup( # move elevator down
                        SetElevator(self.elevator, 0),
                        InstantCommand(lambda: self.wrist.set_wrist_angle(0)),
                    )
            )
                    
            debug_commands.append(
                PrintCommand("Wrist over limit, moving wrist out of the way before running elevator")
            )
        elif not wrist_under_lock_limit() and not elevator_is_over_limit():
            # move wrist down
            commands.append(
                InstantCommand(lambda: self.wrist.set_wrist_angle(0))
            )
            debug_commands.append(
                PrintCommand("Wrist above locking limit, moving wrist down")
            )
        else:
            # run both at the same time
            commands.append(
                    SetElevator(self.elevator, 0),
            )
            
            debug_commands.append(
                PrintCommand("moving elevator down")
            )
        
        commands2.CommandScheduler.schedule(ParallelCommandGroup(
            SequentialCommandGroup(
                *commands
            ),
            SequentialCommandGroup(
                *debug_commands
            )
        )
        )
        
    def isFinished(self) -> bool:
        return True
        
    def end(self, interrupted: bool):
        pass
    
    
class StageNote(commands2.Command):
    
    def __init__(self, wrist: Wrist, elevator: Elevator, intake: Intake):
        self.wrist = wrist
        self.intake = intake
        
    def initialize(self):
        self.wrist.feed_in()
        self.intake.roll_inner_in()
        
    def isFinished(self) -> bool:
        return (not self.intake.note_in_intake )
    
class Shoot(commands2.Command):
    
    def __init__(self, drivetrain: Drivetrain, calculations, odometry: FieldOdometry, elevator: Elevator, wrist: Wrist, target, atPose: Pose2d | None = None):
        self.drivetrain = drivetrain
        self.elevator = elevator
        self.wrist = wrist
        self.odometry = odometry
        self.target = target
        self.atPose = atPose
        self.wrist_at_angle:bool = False
        self.flywheel_at_speed:bool = False
        self.drivetrain_aimed:bool = False
        
    def initialize(self):
        pass
    
    def execute(self):
        pass
    
    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted: bool):
        pass