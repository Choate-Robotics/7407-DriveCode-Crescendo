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
from oi.keymap import Controllers, Keymap
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


        #INTAKE TRIGGERS ----------------
        
        
        #FEEDER TRIGGERS ----------------
        
        button.Trigger(lambda: Robot.wrist.detect_note_first() and not Robot.wrist.detect_note_second() and not robot_states.climbed)\
            .onTrue(
                InstantCommand(lambda: Robot.wrist.set_feed_voltage(config.feeder_voltage_crawl))
            )\
            .onFalse(
                InstantCommand(lambda: Robot.wrist.stop_feed())
            )
        
        button.Trigger(lambda: Robot.wrist.note_in_feeder())\
            .onTrue(
                ParallelCommandGroup(
                    command.ControllerRumbleTimeout(Controllers.OPERATOR_CONTROLLER, config.operator_rumble_time, config.operator_rumble_intensity)
                )
            ).onFalse(
                ParallelCommandGroup(
                command.ControllerRumble(Controllers.OPERATOR_CONTROLLER, 0)
                )
            )
            
        button.Trigger(lambda: Robot.wrist.note_in_feeder() or Robot.intake.detect_note())\
            .onTrue(
                command.ControllerRumble(Controllers.DRIVER_CONTROLLER, config.driver_rumble_intensity),
            ).onFalse(
                command.ControllerRumble(Controllers.DRIVER_CONTROLLER, 0),
            )
    #     #FEEDER TRIGGERS ----------------
        
        
    #     #FLYWHEEL TRIGGERS ----------------
    
        def set_flywheel_state(state: robot_states.FlywheelState):
            if isinstance(state, robot_states.FlywheelState):
                robot_states.flywheel_state = state
    
        button.Trigger(
            lambda: Robot.wrist.note_in_feeder()\
                and not robot_states.flywheel_state == robot_states.FlywheelState.amping\
                and not robot_states.flywheel_state == robot_states.FlywheelState.feeding\
                and not robot_states.flywheel_state == robot_states.FlywheelState.static_feeding
        ).onTrue(
            InstantCommand(lambda: set_flywheel_state(robot_states.FlywheelState.shooting))
        )
        
        button.Trigger(
            lambda: not Robot.wrist.note_in_feeder()\
                and not robot_states.flywheel_state == robot_states.FlywheelState.amping\
                and not robot_states.flywheel_state == robot_states.FlywheelState.feeding\
                and not robot_states.flywheel_state == robot_states.FlywheelState.static_feeding
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
            lambda: robot_states.flywheel_state == robot_states.FlywheelState.feeding\
                and not Keymap.Shooter.FEED_MIDLINE.getAsBoolean()
            )\
            .onTrue(
                command.SetFlywheelShootFeeder(Robot.flywheel, Field.calculations, command.SetFlywheelShootFeeder.Style.dynamic_amp)
            )
            
        button.Trigger(
            lambda: robot_states.flywheel_state == robot_states.FlywheelState.feeding\
                and Keymap.Shooter.FEED_MIDLINE.getAsBoolean()
            )\
            .onTrue(
                command.SetFlywheelShootFeeder(Robot.flywheel, Field.calculations)
            )
            
        button.Trigger(
            lambda: robot_states.flywheel_state == robot_states.FlywheelState.static_feeding
            )\
            .onTrue(
                command.SetFlywheelShootFeeder(Robot.flywheel, Field.calculations, command.SetFlywheelShootFeeder.Style.static)
            )
            
        def set_idle():
            robot_states.flywheel_state = robot_states.FlywheelState.idle

        button.Trigger(
            lambda: robot_states.flywheel_state == robot_states.FlywheelState.idle
                # or robot_states.flywheel_state == robot_states.FlywheelState.released
            ).debounce(1).onTrue(
                ParallelCommandGroup(
                    InstantCommand(set_idle),
                    command.SetFlywheelLinearVelocity(Robot.flywheel, config.idle_flywheel)
                )
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
        ).debounce(0.0).onTrue(command.Shoot(Robot.wrist))
        # SHOOTER TRIGGERS ----------------


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
