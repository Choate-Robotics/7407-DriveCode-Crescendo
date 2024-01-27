from utils import LocalLogger


from commands2 import button, WaitCommand, ParallelRaceGroup, InstantCommand
import config

import command


from robot_systems import Robot, Sensors, Field

from wpilib import DriverStation
import ntcore

log = LocalLogger("IT")


class IT:

    @staticmethod
    def init() -> None:
        log.info("Initializing IT...")

    @staticmethod
    def map_systems():
        log.info("Mapping systems...")
        
        # intake
        button.Trigger(lambda: Robot.intake.get_back_current() > config.intake_roller_current_limit and not Robot.intake.intake_running)\
        .debounce(config.intake_sensor_debounce).onTrue(
                command.RunIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        )
        
        
        # elevator and wrist
        button.Trigger(lambda: Robot.intake.beam_break.get() and Robot.intake.note_in_intake and not Robot.wrist.note_staged)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kStage)
        ).onFalse(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kIdle)
        )
        
        button.Trigger(lambda: Robot.wrist.beam_break.get() and Robot.wrist.note_staged)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAimLow)
        ).onFalse(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kIdle)
        )
        
        
        def stop_limelight_pos():
            Sensors.limelight.cam_pos_moving = True
            
        def start_limelight_pos():
            Sensors.limelight.cam_pos_moving = False
            
        def setFieldRed():
            Field.POI.setRed()
            
        def setFieldBlue():
            Field.POI.setBlue()       
        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1)\
        #     .onTrue(InstantCommand(stop_limelight_pos))\
        #     .onFalse(InstantCommand(start_limelight_pos))
        
        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1)\
        #     .onTrue(InstantCommand(stop_limelight_pos))\
        #     .onFalse(InstantCommand(start_limelight_pos))


