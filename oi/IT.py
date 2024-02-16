
from utils import LocalLogger



from commands2 import button, ParallelDeadlineGroup, WaitCommand, ParallelRaceGroup, InstantCommand, PrintCommand
import command
import config, math

# ADD ROBOT IN TO THE IMPORT FROM ROBOT_SYSTEMS LATER
from utils import LocalLogger


from robot_systems import Robot, Sensors, Field


log = LocalLogger("IT")


class IT:
    @staticmethod
    def init() -> None:
        log.info("Initializing IT...")

    @staticmethod
    def map_systems():
        log.info("Mapping systems...")
        
        def lock_giraffe():
            Robot.elevator.lock()
            Robot.wrist.lock()
            
        def unlock_giraffe():
            Robot.elevator.unlock()
            Robot.wrist.unlock()

        
        # If outer roller detects a note, run the intake in
        button.Trigger(lambda: Robot.intake.get_outer_current() > config.intake_roller_current_limit and not Robot.intake.intake_running)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.RunIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        )
        
        # # if note in intake and wrist, after certain time, eject 
        # # button.Trigger(lambda: Robot.intake.note_in_intake)\
        # #     .debounce(config.double_note_timeout).onTrue(
        # #         command.EjectIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        # #     )
        
        # # if note in intake and index ready to receive, pass note to index
        button.Trigger(lambda: Robot.intake.note_in_intake and not Robot.wrist.note_staged)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.StageNote(Robot.elevator, Robot.wrist, Robot.intake)
        )
        
        # # if note in feeder, run flywheel and wrist to aim
        button.Trigger(lambda: Robot.wrist.note_staged)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAimLow).withTimeout(5)\
                .andThen(command.SetFlywheelLinearVelocity(Robot.flywheel, 30)).withTimeout(5)\
                .andThen(command.PassNote(Robot.wrist))
        )
        
        button.Trigger(lambda: Robot.intake.note_in_intake)\
            .onTrue(
                command.SetFlywheelLinearVelocity(Robot.flywheel, 10)
            )
            
        button.Trigger(lambda: Robot.wrist.note_staged)\
            .onTrue(
                command.SetFlywheelLinearVelocity(Robot.flywheel, 25)
            )
            
        button.Trigger(lambda: not Robot.wrist.note_staged and not Robot.intake.note_in_intake)\
            .onTrue(
                command.SetFlywheelLinearVelocity(Robot.flywheel, 0)
            )
        
        
        # # odom stage
        # button.Trigger(lambda: Field.odometry.getPose().translation().distance(Field.POI.Coordinates.Structures.Obstacles.kStage.getTranslation()) < config.stage_distance_threshold\
        #     and (Robot.elevator.get_length() > config.elevator_stage_max or Robot.wrist.get_wrist_angle() < config.wrist_stage_max))\
        #     .debounce(config.odometry_debounce).onTrue(
        #         command.GiraffeLock(Robot.elevator, Robot.wrist)
        #     ).onFalse(InstantCommand(unlock_giraffe))
        
        
        def stop_limelight_pos():
            Sensors.limelight.cam_pos_moving = True

        def start_limelight_pos():
            Sensors.limelight.cam_pos_moving = False


        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1)\
        #     .onTrue(InstantCommand(stop_limelight_pos))\
        #     .onFalse(InstantCommand(start_limelight_pos))

        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1)\
        #     .onTrue(InstantCommand(stop_limelight_pos))\
        #     .onFalse(InstantCommand(start_limelight_pos))


        # COMMENTED OUT BECAUSE INTAKE IS NOT PART OF THE CODE BASE AT THIS TIME
        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1).onTrue(
        #     InstantCommand(stop_limelight_pos)
        # ).onFalse(InstantCommand(start_limelight_pos))
