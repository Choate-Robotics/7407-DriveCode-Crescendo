from __future__ import annotations
import utils
import constants, config

from toolkit.command import SubsystemCommand
from subsystem import Flywheel
from sensors import TrajectoryCalculator
from units.SI import meters_per_second
import robot_states as states
from enum import Enum

class SetFlywheelLinearVelocity(SubsystemCommand[Flywheel]):
    """
    Set velocity of both top and bottom flywheels.
    param velocity: speed to set flywheels to in m/s
    """
    def __init__(self, subsystem: Flywheel, velocity: meters_per_second):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.velocity = velocity

    def initialize(self):
        self.subsystem.set_velocity_linear(self.velocity)

    def execute(self):
        print(self.subsystem.get_velocity_linear(), 'Current velocity')

    def isFinished(self):
        return self.subsystem.within_velocity_linear(self.velocity, 2)

    def end(self, interrupted: bool):
        print(self.subsystem.get_velocity_linear(), 'Final velocity')


class SetFlywheelVelocityIndependent(SubsystemCommand[Flywheel]):
    """
    Set velocity of top and bottom flywheel independently.
    param velocity: tuple. first value is top flywheel, second is bottom flywheel
    """
    def __init__(self, subsystem: Flywheel, velocity: tuple[meters_per_second, meters_per_second]):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.velocity:tuple[meters_per_second, meters_per_second] = velocity

    def initialize(self):
        self.subsystem.set_velocity_linear(self.velocity[0], 1)
        self.subsystem.set_velocity_linear(self.velocity[1], 2)

    def execute(self):
        pass

    def isFinished(self):
        return (
            self.subsystem.within_velocity_linear(self.velocity[0], 0.1, 1)
            and
            self.subsystem.within_velocity_linear(self.velocity[1], 0.1, 2)
        )

    def end(self, interrupted: bool):
        pass
    
    
class SetFlywheelShootSpeaker(SubsystemCommand[Flywheel]):
    
    def __init__(self, subsystem: Flywheel, trajectory: TrajectoryCalculator):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.traj = trajectory
        
    def initialize(self):
        self.subsystem.set_velocity_linear(config.v0_flywheel_minimum, 1)
        self.subsystem.set_velocity_linear(config.v0_flywheel_minimum, 2)
        
    
    def execute(self):
        distance = self.traj.get_distance_to_target()
        
        tolerance = self.traj.get_flywheel_shot_tolerance(distance)
        
        states.flywheel_tolerance = tolerance
        
        speed = self.traj.get_flywheel_speed(distance)
        
        self.subsystem.set_velocity_linear(speed, 1)
        self.subsystem.set_velocity_linear(speed, 2)
        
        # self.subsystem.motor_1.set_raw_output(1)
        
        
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        states.flywheel_tolerance = config.flywheel_min_shot_tolerance


class SetFlywheelShootFeeder(SubsystemCommand[Flywheel]):
    
    class Style(Enum):
        
        dynamic = 0
        static = 1
        dynamic_amp = 2
    
    def __init__(self, subsystem: Flywheel, trajectory: TrajectoryCalculator,style:Style=Style.dynamic):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.traj = trajectory
        self.style = style
        
    def initialize(self):
        self.subsystem.set_velocity_linear(config.flywheel_feed_speed_min, 1)
        self.subsystem.set_velocity_linear(config.flywheel_feed_speed_min, 2)
        
    
    def execute(self):
        distance = (
            self.traj.get_distance_to_feed_zone()
            if self.style == SetFlywheelShootFeeder.Style.dynamic
            else self.traj.get_distance_to_feed_zone_static(True)
            if self.style == SetFlywheelShootFeeder.Style.static
            else self.traj.get_distance_to_feed_zone(force_amp=True)
            )
        
        speed = self.traj.get_flywheel_speed_feed(distance)
        
        self.subsystem.set_velocity_linear(speed, 1)
        self.subsystem.set_velocity_linear(speed, 2)
        
        # self.subsystem.motor_1.set_raw_output(1)
        
        
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        pass