from subsystem import Elevator, Wrist, Intake, Drivetrain

import logging
from command import DriveSwerveCustom, SetElevator, ZeroElevator
import wpilib, config, constants
import commands2
from commands2 import WaitUntilCommand, WaitCommand, ParallelCommandGroup, SequentialCommandGroup, InstantCommand, PrintCommand



class Giraffe(commands2.Command):
    
    def __init__(self, elevator: Elevator, wrist: Wrist):
        self.elevator = elevator
        self.wrist = wrist
    
    def initialize(self):
        commands = []
        
        
        
        
        commands2.CommandScheduler.schedule(commands)
        
    def isFinished(self) -> bool:
        return True
        
    def end(self, interrupted: bool):
        pass
    
    