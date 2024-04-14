from utils import LocalLogger

import commands2
import command, config, constants
from robot_systems import Robot, Sensors, Field
from oi.keymap import Keymap
import wpilib
from math import radians
import robot_states as states
log = LocalLogger("OI")


class OI:

    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")

        Keymap.Drivetrain.RESET_GYRO.onTrue(
            command.DrivetrainZero(Robot.drivetrain)) \
            .onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Shooter.AIM.and_(lambda: not states.flywheel_state == states.FlywheelState.amping).whileTrue(
            command.DriveSwerveAim(Robot.drivetrain, Field.calculations)
        ).onFalse(
            command.DriveSwerveCustom(Robot.drivetrain)
        )


        # Keymap.Shooter.AIM_DRIVETRAIN_RIGHT.and_(lambda: not states.flywheel_state == states.FlywheelState.amping).whileTrue(
        #     command.DriveSwerveAim(Robot.drivetrain, Field.calculations)
        # ).onFalse(
        #     command.DriveSwerveCustom(Robot.drivetrain)
        # )
        
        Keymap.Shooter.AMP\
            .whileTrue(
                commands2.ConditionalCommand(
                    command.DriveSwerveHoldRotationIndef(Robot.drivetrain, radians(-90)),
                    command.DriveSwerveHoldRotationIndef(Robot.drivetrain, radians(90)),
                    lambda: config.active_team == config.Team.RED
                )
            ).onFalse(
                command.DriveSwerveCustom(Robot.drivetrain)
            )
        
        Keymap.Shooter.AIM.and_(lambda: Robot.wrist.detect_note_second())\
            .and_(lambda: not states.flywheel_state == states.FlywheelState.amping)\
            .whileTrue(
                command.AimWrist(Robot.wrist, Field.calculations)
            ).onFalse(
                commands2.WaitCommand(0.5).andThen(
                command.SetWristIdle(Robot.wrist)
                )
            )

        Keymap.Drivetrain.X_MODE.onTrue(
            commands2.InstantCommand(lambda: Robot.drivetrain.x_mode())
        )

        Keymap.Intake.INTAKE_IN.whileTrue(
            command.SetElevator(Robot.elevator, config.Giraffe.kIdle.height).alongWith(
            command.IntakeStageNote(Robot.wrist, Robot.intake))
        ).onFalse(
            command.IntakeStageIdle(Robot.wrist, Robot.intake)
        )

        Keymap.Intake.INTAKE_OUT.onTrue(
            command.EjectIntake(Robot.intake)
        ).onFalse(
            command.IntakeIdle(Robot.intake)
        )
        
        # Keymap.Intake.AUTO_INTAKE.onTrue(
        #     command.AutoPickupNote(Robot.drivetrain, Robot.wrist, Robot.intake, Sensors.limelight_intake)
        # ).onFalse(
        #     commands2.ParallelCommandGroup(
        #         # command.IntakeStageIdle(Robot.wrist, Robot.intake),
        #         commands2.ConditionalCommand(
        #             command.IntakeStageNote(Robot.wrist, Robot.intake).andThen(
        #                 command.IntakeStageIdle(Robot.wrist, Robot.intake)),
        #             command.IntakeStageIdle(Robot.wrist, Robot.intake),
        #             lambda: Robot.intake.detect_note() or Robot.wrist.detect_note_first()
        #         ),
        #         command.DriveSwerveCustom(Robot.drivetrain)
        #     )
        # )
        
        Keymap.Shooter.FEED_SHOT.onTrue(
            command.DriveSwerveAim(Robot.drivetrain, Field.calculations, command.DriveSwerveAim.Target.feed, False)
        ).onFalse(
            command.DriveSwerveCustom(Robot.drivetrain)
        )
        
        Keymap.Shooter.FEED_SHOT.onTrue(
            command.AimWrist(Robot.wrist, Field.calculations, command.AimWrist.Target.feed)
        ).onFalse(
                commands2.WaitCommand(0.5).andThen(
                command.SetWristIdle(Robot.wrist)
                )
            )
        # Keymap.Intake.AUTO_INTAKE.onTrue(
        #     command.DriveSwerveNoteLineup(Robot.drivetrain, Sensors.limelight_intake)
        # ).onFalse(
        #     command.DriveSwerveCustom(Robot.drivetrain)
        # )

        Keymap.Elevator.ELEVATOR_HIGH.onTrue(
            command.SetElevator(Robot.elevator, config.Giraffe.kElevatorHigh.height)
        ).onFalse(
            command.SetElevator(Robot.elevator, config.Giraffe.kElevatorLow.height)
        )

        Keymap.Elevator.ELEVATOR_MID.onTrue(
            command.SetElevator(Robot.elevator, config.Giraffe.kElevatorMid.height)
        ).onFalse(
            command.SetElevator(Robot.elevator, config.Giraffe.kElevatorLow.height)
        )

        Keymap.Elevator.ELEVATOR_LOW.onTrue(
            command.SetElevator(Robot.elevator, config.Giraffe.kElevatorLow.height)
        )

        def set_amping():
            states.flywheel_state = states.FlywheelState.amping

        def set_released():
            states.flywheel_state = states.FlywheelState.released
            
        Keymap.Elevator.AMP.whileTrue(
            command.Amp(Robot.elevator, Robot.wrist)
        ).onFalse(
            command.SetElevator(Robot.elevator, config.Giraffe.kIdle.height).alongWith(
                command.SetWristIdle(Robot.wrist)
            )
        )
        
        
        Keymap.Elevator.AMP.onTrue(
            commands2.InstantCommand(lambda: set_amping())
        ).onFalse(
            commands2.InstantCommand(lambda: set_released())
        )
        
        def set_feeding():
            states.flywheel_state = states.FlywheelState.feeding
            
            
        Keymap.Shooter.FEED_SHOT.onTrue(
            commands2.InstantCommand(lambda: set_feeding())
        ).onFalse(
            commands2.InstantCommand(lambda: set_released())
        )

        Keymap.Shooter.SET_WRIST_SUBWOOFER.onTrue(
            command.SetWristIdle(Robot.wrist)
        )

        Keymap.Shooter.ENABLE_AIM_WRIST.onTrue(
            command.AimWrist(Robot.wrist, Field.calculations)
        )

        Keymap.Shooter.ENABLE_AIM_WRIST_OPERATOR.onTrue(
            command.AimWrist(Robot.wrist, Field.calculations)
        )

        Keymap.Feeder.DUMP_NOTE.onTrue(
            command.PassNote(Robot.wrist),
        )

        Keymap.Feeder.FEED.and_(lambda: not Robot.wrist.note_detected()).onTrue(
            commands2.InstantCommand(lambda: Robot.wrist.feed_in())
        ).onFalse(
            commands2.InstantCommand(lambda: Robot.wrist.stop_feed())
        )

        Keymap.Feeder.UNFEED.onTrue(
            commands2.InstantCommand(lambda: Robot.wrist.feed_out())
        ).onFalse(
            commands2.InstantCommand(lambda: Robot.wrist.stop_feed())
        )

        def climb_ready():
            states.ready_to_climb = True

        def climb_not_ready():
            states.ready_to_climb = False

        def start_climbing():
            states.climbing = True

        def stop_climbing():
            states.climbing = False

        def climbed():
            states.climbed = True

        def not_climbed():
            states.climbed = False

        Keymap.Climb.CLIMB_UP.and_(lambda: not states.climbing).onTrue(
            commands2.InstantCommand(lambda: climb_ready()).andThen(
                commands2.InstantCommand(lambda: start_climbing()).alongWith(
                    command.EnableClimb(Robot.elevator, Robot.wrist, Robot.intake)
                )
            )
        )

        Keymap.Climb.UNDO_CLIMB_UP.and_(lambda: states.climbing).onTrue(
            commands2.InstantCommand(lambda: stop_climbing())
            .alongWith(commands2.InstantCommand(lambda: climb_not_ready()).alongWith(
                command.UndoClimb(Robot.elevator, Robot.wrist, Robot.intake)
            ))
        )

        Keymap.Climb.CLIMB_DOWN.and_(lambda: states.climbing).onTrue(
            command.ClimbDown(Robot.elevator, Robot.wrist).alongWith(
                commands2.InstantCommand(lambda: climbed())
            )
        )

        Keymap.Climb.TRAP.and_(lambda: states.climbed).onTrue(command.ScoreTrap(Robot.elevator, Robot.wrist))

        def set_manual_flywheel():
            states.flywheel_manual = True

        def set_auto_flywheel():
            states.flywheel_manual = False

