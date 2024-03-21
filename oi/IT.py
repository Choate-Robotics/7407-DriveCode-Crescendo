import math  # noqa

from commands2 import ParallelDeadlineGroup  # noqa
from commands2 import ParallelRaceGroup  # noqa
from commands2 import PrintCommand  # noqa
from commands2 import InstantCommand, WaitCommand, button
from wpimath.filter import Debouncer  # noqa
from wpimath.geometry import Pose3d, Rotation3d, Transform3d  # noqa

from commands2 import button, ParallelDeadlineGroup, WaitCommand, ParallelRaceGroup, InstantCommand, PrintCommand, ParallelCommandGroup
import command
import config
import robot_states
from oi.keymap import Controllers
from robot_systems import Field, Robot, Sensors

# ADD ROBOT IN TO THE IMPORT FROM ROBOT_SYSTEMS LATER
from utils import LocalLogger

log = LocalLogger("IT")


class IT:
    @staticmethod
    def init() -> None:
        log.info("Initializing IT...")

    @staticmethod
    def map_systems():
        log.info("Mapping systems...")

        # INTAKE TRIGGERS ----------------
        # If outer roller detects a note, run the intake in
        # button.Trigger(lambda: Robot.intake.get_outer_current() >
        # config.intake_roller_current_limit and not Robot.intake.intake_running)\
        # .debounce(config.intake_sensor_debounce).onTrue(
        #     command.RunIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        # )

        # # If note in intake and wrist, after certain time, eject
        # button.Trigger(lambda: Robot.intake.note_in_intake)\
        #     .debounce(config.double_note_timeout).onTrue(
        #         command.EjectIntake(Robot.intake).withTimeout(config.intake_timeout).andThen(command.IntakeIdle(Robot.intake))
        #     )

        #INTAKE TRIGGERS ----------------
        
        
        #FEEDER TRIGGERS ----------------
        
        button.Trigger(lambda: Robot.wrist.detect_note_first() and not Robot.wrist.detect_note_second()).and_(lambda: not robot_states.climbed)\
            .onTrue(
                InstantCommand(lambda: Robot.wrist.set_feed_voltage(config.feeder_voltage_crawl))
            )\
            .onFalse(
                InstantCommand(lambda: Robot.wrist.stop_feed())
            )
        
        button.Trigger(lambda: Robot.wrist.note_in_feeder())\
            .onTrue(
                ParallelCommandGroup(
                    command.ControllerRumble(Controllers.DRIVER_CONTROLLER, config.driver_rumble_intensity),
                    command.ControllerRumbleTimeout(Controllers.OPERATOR_CONTROLLER, config.operator_rumble_time, config.operator_rumble_intensity)
                )
            ).onFalse(
                ParallelCommandGroup(
                command.ControllerRumble(Controllers.DRIVER_CONTROLLER, 0),
                command.ControllerRumble(Controllers.OPERATOR_CONTROLLER, 0)
                )
            )
    #     #FEEDER TRIGGERS ----------------
        
        
    #     #FLYWHEEL TRIGGERS ----------------
    
        def set_flywheel_state(state: robot_states.FlywheelState):
            if isinstance(state, robot_states.FlywheelState):
                robot_states.flywheel_state = state
    
        button.Trigger(
            lambda: Robot.wrist.note_in_feeder()\
                and not robot_states.flywheel_state == robot_states.FlywheelState.amping
        ).onTrue(
            InstantCommand(lambda: set_flywheel_state(robot_states.FlywheelState.shooting))
        )
        
        button.Trigger(
            lambda: not Robot.wrist.note_in_feeder()\
                and not robot_states.flywheel_state == robot_states.FlywheelState.amping
        ).onTrue(
            InstantCommand(lambda: set_flywheel_state(robot_states.FlywheelState.idle))
        )

        # if note in feeder, spin to set shot velocity
        button.Trigger(
            lambda: robot_states.flywheel_state == robot_states.FlywheelState.shooting
            ).onTrue(
                command.SetFlywheelShootSpeaker(Robot.flywheel, Field.calculations),
                # command.SetFlywheelVelocityIndependent(Robot.flywheel, (config.v0_flywheel - 1, config.v0_flywheel + 1))
            )
        button.Trigger(
            lambda: robot_states.flywheel_state == robot_states.FlywheelState.amping
            )\
            .onTrue(
                command.SetFlywheelVelocityIndependent(Robot.flywheel, (config.flywheel_amp_speed, 5))
            )

        button.Trigger(
            lambda: robot_states.flywheel_state == robot_states.FlywheelState.idle
            ).debounce(1).onTrue(
                command.SetFlywheelLinearVelocity(Robot.flywheel, config.idle_flywheel)
            )
            
    #     #FLYWHEEL TRIGGERS ----------------
        
        
        #SHOOTER TRIGGERS ----------------
        
        def reset_shooter():
            Robot.wrist.set_note_not_staged()
            Robot.drivetrain.ready_to_shoot = False
            Robot.flywheel.ready_to_shoot = False

        button.Trigger(
            lambda: Robot.wrist.ready_to_shoot
            and Robot.drivetrain.ready_to_shoot
            and Robot.flywheel.ready_to_shoot
            and not Robot.elevator.elevator_moving
        ).debounce(0.005).onTrue(command.Shoot(Robot.wrist))
        # SHOOTER TRIGGERS ----------------

        #     #ODOMETRY TRIGGERS ----------------

        #     def lock_giraffe():
        #         Robot.elevator.lock()
        #         Robot.wrist.lock()

        #     def unlock_giraffe():
        #         Robot.elevator.unlock()
        #         Robot.wrist.unlock()

        #     # # if close to stage, lock giraffe
        #     # button.Trigger(lambda: Field.odometry.getPose().translation()
        #     .distance(Field.POI.Coordinates.Structures.Obstacles.kStage
        #     .getTranslation()) < config.stage_distance_threshold\
        #     #     and (Robot.elevator.get_length() > config.elevator_stage_max
        #     or Robot.wrist.get_wrist_angle() < config.wrist_stage_max))\
        #     #     .debounce(config.odometry_debounce).onTrue(
        #     #         command.GiraffeLock(Robot.elevator, Robot.wrist)
        #     #     ).onFalse(InstantCommand(unlock_giraffe))
        #     # ODOMETRY TRIGGERS ----------------

        #     # LIMELIGHT TRIGGERS ----------------
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

        # # if elevator is moving, disable limelight
        # button.Trigger(lambda: Robot.elevator.elevator_moving).debounce(0.1)\
        #     .onTrue(InstantCommand(stop_limelight_pos))\
        #     .onFalse(InstantCommand(start_limelight_pos))

    #     #LIMELIGHT TRIGGERS ----------------
