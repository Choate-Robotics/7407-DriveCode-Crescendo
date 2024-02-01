
from utils import LocalLogger



from commands2 import button, ParallelDeadlineGroup, WaitCommand, ParallelRaceGroup, InstantCommand
import command
import config


# import command
# import config
# ADD ROBOT IN TO THE IMPORT FROM ROBOT_SYSTEMS LATER
from robot_systems import Field, Sensors
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

        
        # intake
        button.Trigger(lambda: Robot.intake.get_back_current() > config.intake_roller_current_limit and not Robot.intake.intake_running)\
        .debounce(config.intake_sensor_debounce).onTrue(
                command.RunIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        )
        
        button.Trigger(lambda: Robot.intake.note_in_intake and not Robot.wrist.note_staged)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.StageNote(Robot.elevator, Robot.wrist, Robot.intake).withTimeout(config.stage_timeout).andThen(command.IntakeIdle(Robot.intake))
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
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim)
        ).onFalse(
            command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kIdle)
        )
        
        # odom
        button.Trigger(lambda: Field.odometry.getPose().translation().distance(Field.POI.Coordinates.Structures.Obstacles.kStage.getTranslation()) < config.stage_distance_threshold\
            and (Robot.elevator.get_length() > config.elevator_stage_max or Robot.wrist.get_wrist_angle() < config.wrist_stage_max))\
            .debounce(config.odometry_debounce).onTrue(
                command.GiraffeLock(Robot.elevator, Robot.wrist)
            ).onFalse(InstantCommand(lambda: Robot.elevator.unlock()))
        
        
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


        # COMMENTED OUT BECAUSE INTAKE IS NOT PART OF THE CODE BASE AT THIS TIME
        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1).onTrue(
        #     InstantCommand(stop_limelight_pos)
        # ).onFalse(InstantCommand(start_limelight_pos))
