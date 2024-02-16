
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

        

        button.Trigger(lambda: Robot.intake.get_outer_current() > config.intake_roller_current_limit and not Robot.intake.intake_running)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.RunIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        )
        
        # # if note in intake and wrist, after certain time, eject 
        # # button.Trigger(lambda: Robot.intake.note_in_intake)\
        # #     .debounce(config.double_note_timeout).onTrue(
        # #         command.EjectIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        # #     )
        
        # # if note in intake and index ready to recieve, run in TODO: add wrist/index to this
        button.Trigger(lambda: Robot.intake.note_in_intake and not Robot.wrist.note_staged)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.SetWrist(Robot.wrist, math.radians(55)).andThen(
            command.PassIntakeNote(Robot.intake).alongWith(command.FeedIn(Robot.wrist))\
                .andThen(InstantCommand(Robot.intake.remove_note))
            )
        )
        
        # # if note in intake and index ready to recieve, run in TODO: add wrist/index to this
        button.Trigger(lambda: Robot.wrist.note_staged)\
        .debounce(config.intake_sensor_debounce).onTrue(
            command.SetWrist(Robot.wrist, math.radians(10))\
                .andThen(command.SetFlywheelLinearVelocity(Robot.flywheel, 30)).withTimeout(5)\
                .andThen(command.PassNote(Robot.wrist))
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


        # COMMENTED OUT BECAUSE INTAKE IS NOT PART OF THE CODE BASE AT THIS TIME
        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1).onTrue(
        #     InstantCommand(stop_limelight_pos)
        # ).onFalse(InstantCommand(start_limelight_pos))
