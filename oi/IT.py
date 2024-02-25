
from utils import LocalLogger



from commands2 import button, ParallelDeadlineGroup, WaitCommand, ParallelRaceGroup, InstantCommand, PrintCommand, ParallelCommandGroup, WaitUntilCommand, SequentialCommandGroup
import command
import config, math
from wpimath.geometry import Pose3d, Rotation3d, Transform3d
# ADD ROBOT IN TO THE IMPORT FROM ROBOT_SYSTEMS LATER
from utils import LocalLogger
from oi.keymap import Controllers


from robot_systems import Robot, Sensors, Field


log = LocalLogger("IT")


class IT:
    @staticmethod
    def init() -> None:
        log.info("Initializing IT...")

    @staticmethod
    def map_systems():
        log.info("Mapping systems...")
        
        

        #INTAKE TRIGGERS ----------------
        # If outer roller detects a note, run the intake in
        # button.Trigger(lambda: Robot.intake.get_outer_current() > config.intake_roller_current_limit and not Robot.intake.intake_running)\
        # .debounce(config.intake_sensor_debounce).onTrue(
        #     command.RunIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        # )
        
        # # If note in intake and wrist, after certain time, eject 
        # button.Trigger(lambda: Robot.intake.note_in_intake)\
        #     .debounce(config.double_note_timeout).onTrue(
        #         command.EjectIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        #     )
        
        # # if note in intake and index ready to receive, pass note to index
        button.Trigger(lambda: Robot.intake.detect_note()).debounce(0).onTrue(
            InstantCommand(lambda: Robot.intake.add_note())
        ).onFalse(
            InstantCommand(lambda: Robot.intake.remove_note())
        )
        
        
        button.Trigger(lambda: Robot.intake.note_in_intake and not Robot.wrist.note_staged)\
        .onTrue(
            command.StageNote(Robot.elevator, Robot.wrist, Robot.intake)
        )
        #INTAKE TRIGGERS ----------------
        
        
        #FEEDER TRIGGERS ----------------
        
        button.Trigger(lambda: Robot.wrist.note_detected()).onTrue(
            InstantCommand(lambda: Robot.wrist.set_note_staged())
        ).onFalse(
            InstantCommand(lambda: Robot.wrist.set_note_not_staged())
        )
        
        # # if note in feeder, run flywheel and wrist to aim
        # button.Trigger(lambda: Robot.wrist.detect_note_first() or Robot.wrist.detect_note_second())\
        # .onTrue(
        #     command.AimWrist(Robot.wrist, Field.calculations)#)
        # ).debounce(.1).onFalse(
        #     command.SetWrist(Robot.wrist, math.radians(58.5))
        # )
        
        button.Trigger(lambda: Robot.wrist.detect_note_first() or Robot.wrist.detect_note_second())\
            .onTrue(
                InstantCommand(lambda: Controllers.DRIVER_CONTROLLER.setRumble(
                    Controllers.DRIVER_CONTROLLER.RumbleType.kBothRumble,
                    1
                ))
            ).onFalse(
                InstantCommand(lambda: Controllers.DRIVER_CONTROLLER.setRumble(
                    Controllers.DRIVER_CONTROLLER.RumbleType.kBothRumble,
                    0
                ))
            )
            
        button.Trigger(lambda: Robot.wrist.note_staged)\
            .onTrue(
                InstantCommand(lambda: Controllers.OPERATOR_CONTROLLER.setRumble(
                    Controllers.OPERATOR_CONTROLLER.RumbleType.kBothRumble,
                    1
                ))
            ).onFalse(
                InstantCommand(lambda: Controllers.OPERATOR_CONTROLLER.setRumble(
                    Controllers.OPERATOR_CONTROLLER.RumbleType.kBothRumble,
                    0
                ))
            )
        #FEEDER TRIGGERS ----------------
        
        
        #FLYWHEEL TRIGGERS ----------------
            
        # if note in feeder, spin to set shot velocity
        button.Trigger(lambda: Robot.wrist.detect_note_first() or Robot.wrist.detect_note_second())\
            .onTrue(
                command.SetFlywheelLinearVelocity(Robot.flywheel, config.v0_flywheel)
                # command.SetFlywheelVelocityIndependent(Robot.flywheel, (config.v0_flywheel - 1, config.v0_flywheel + 1))
            ).onFalse(
                command.SetFlywheelLinearVelocity(Robot.flywheel, config.idle_flywheel)
            )
            
            
        #FLYWHEEL TRIGGERS ----------------
        
        
        #SHOOTER TRIGGERS ----------------
        button.Trigger(lambda: Robot.wrist.ready_to_shoot and Robot.drivetrain.ready_to_shoot and Robot.flywheel.ready_to_shoot)\
            .onTrue(
                command.Shoot(Robot.wrist)
            )
        #SHOOTER TRIGGERS ----------------
        
        #ODOMETRY TRIGGERS ----------------
        
        def lock_giraffe():
            Robot.elevator.lock()
            Robot.wrist.lock()
            
        def unlock_giraffe():
            Robot.elevator.unlock()
            Robot.wrist.unlock()
        
        # # if close to stage, lock giraffe
        # button.Trigger(lambda: Field.odometry.getPose().translation().distance(Field.POI.Coordinates.Structures.Obstacles.kStage.getTranslation()) < config.stage_distance_threshold\
        #     and (Robot.elevator.get_length() > config.elevator_stage_max or Robot.wrist.get_wrist_angle() < config.wrist_stage_max))\
        #     .debounce(config.odometry_debounce).onTrue(
        #         command.GiraffeLock(Robot.elevator, Robot.wrist)
        #     ).onFalse(InstantCommand(unlock_giraffe))
        #ODOMETRY TRIGGERS ----------------
        
        #LIMELIGHT TRIGGERS ----------------
        def stop_limelight_pos():
            Sensors.limelight_front.enable_moving()
            Sensors.limelight_back.enable_moving()

        def start_limelight_pos():
            
            elevator_height = Robot.elevator.get_length()
            z_pose = Transform3d(0, 0, elevator_height, Rotation3d(0, 0, 0))
            
            front_pose = Sensors.limelight_front.get_cam_pose()
            Sensors.limelight_front.disable_moving(front_pose + z_pose)
            
            back_pose = Sensors.limelight_back.get_cam_pose()
            Sensors.limelight_back.disable_moving(back_pose + z_pose)
            

        # if elevator is moving, disable limelight
        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1)\
        #     .onTrue(InstantCommand(stop_limelight_pos))\
        #     .onFalse(InstantCommand(start_limelight_pos))
            
        #LIMELIGHT TRIGGERS ----------------
