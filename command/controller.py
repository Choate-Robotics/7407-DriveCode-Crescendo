from subsystem import Elevator, Wrist, Intake, Drivetrain

import logging
from command import DriveSwerveCustom, SetElevator, ZeroElevator
import wpilib, config, constants
import commands2
from commands2 import WaitUntilCommand, WaitCommand, ParallelCommandGroup, SequentialCommandGroup, InstantCommand, PrintCommand, ParallelDeadlineGroup, RunCommand



class Giraffe(commands2.Command):
    
    def __init__(self, elevator: Elevator, wrist: Wrist, target: config.GiraffePos):
        self.elevator = elevator
        self.wrist = wrist
        self.target = target
    
    
    def initialize(self):
        
        def wrist_is_over_limit():
            return self.wrist.get_wrist_angle() > config.wrist_rotation_limit
        
        def wrist_target_over_limit():
            return self.target.wrist_angle > config.wrist_rotation_limit
        
        def elevator_is_over_limit():
            return self.elevator.get_length() > config.elevator_wrist_limit
        
        def elevator_target_over_limit():
            return self.target.height > config.elevator_wrist_limit
        
        commands = []
        
        if self.target.height == None or self.target.wrist_angle == None:
            return
        
        if not elevator_target_over_limit() and wrist_target_over_limit():
            return # exceeds limits of wrist and elevator constraints
        
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
        elif wrist_target_over_limit() and elevator_is_over_limit():
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
        else:
            # run both at the same time
            commands.append(
                ParallelCommandGroup(
                    SetElevator(self.elevator, self.target.height),
                    InstantCommand(lambda: self.wrist.set_wrist_angle(self.target.wrist_angle)),
                )
            )
        
        commands2.CommandScheduler.schedule(*commands)
        
    def isFinished(self) -> bool:
        return True
        
    def end(self, interrupted: bool):
        pass
    
    