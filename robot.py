import math  # noqa
import time
from math import degrees, pi, radians  # noqa

import commands2
import ntcore
import wpilib
from wpilib import SmartDashboard  # noqa
from wpimath.geometry import Pose2d, Rotation2d  # noqa

import autonomous
import command
import config
import constants
import sensors  # noqa
import subsystem  # noqa
import utils
from oi.IT import IT
from oi.OI import OI
from robot_systems import (  # noqa
    Field,
    LEDs,
    Pneumatics,
    PowerDistribution,
    Robot,
    Sensors,
)
from toolkit.subsystem import Subsystem
from units.SI import inches_to_meters
from utils import CAN_delay
import robot_states as states


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.log = utils.LocalLogger("Robot")
        self.nt = ntcore.NetworkTableInstance.getDefault()

        # Updates networktables at 100hz (dont use unless graphing PID values)
        # self.nt.flush()

        self.scheduler = commands2.CommandScheduler.getInstance()

    def handle(self, func, *args, **kwargs):
        try:
            func(*args, **kwargs)
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString(func.__name__, str(e))

            if config.DEBUG_MODE:
                raise e

    def robotInit(self):
        self.log._robot_log_setup()

        if config.DEBUG_MODE:
            self.log.setup("WARNING: DEBUG MODE IS ENABLED")

        self.scheduler.setPeriod(config.period)

        self.auto_selection = wpilib.SendableChooser()
        # self.auto_selection.addOption("Test", autonomous.drive_straight)
        self.auto_selection.setDefaultOption("Five Note", autonomous.four_note_middle)
        # self.auto_selection.addOption("Two Notes", autonomous.two_note)
        self.auto_selection.addOption("Source Midline Auto", autonomous.mid_notes)
        self.auto_selection.addOption("Alt Source Midline Auto", autonomous.mid_notes_2)
        # self.auto_selection.addOption("Four Notes", autonomous.four_note)
        self.auto_selection.addOption("Amp Side", autonomous.left_four_note)
        self.auto_selection.addOption("Speaker and Leave", autonomous.speaker_shoot_leave)
        # self.auto_selection.addOption("Do Nothing")
        self.auto_selection.addOption('Alt Amp Side', autonomous.left_four_note_reverse)
        # self.auto_selection.addOption("Bobcats counter auto", autonomous.mid_notes_2)
        # self.auto_selection.addOption("Right Three Notes", autonomous.right_three_note)
        # self.auto_selection.addOption("Five Notes", autonomous.five_note)
        # self.auto_selection.addOption("Amp Three Piece", autonomous.amp_auto)
        # self.auto_selection.addOption("Shoot Note", autonomous.aim_shoot_auto)
        # self.auto_selection.addOption("SQUARE of death", autonomous.square)

        wpilib.SmartDashboard.putData("Auto", self.auto_selection)

        self.team_selection = wpilib.SendableChooser()
        self.team_selection.addOption("Blue", config.Team.BLUE)
        self.team_selection.setDefaultOption("Red", config.Team.RED)

        wpilib.SmartDashboard.putData("Team", self.team_selection)

        self.log.info(f"Scheduler period set to {config.period} seconds")

        self.note_1_selection = wpilib.SendableChooser()
        self.note_1_selection.setDefaultOption("Far", config.NoteSelect.FAR)
        self.note_1_selection.addOption("Mid", config.NoteSelect.MID)
        self.note_1_selection.addOption("Center", config.NoteSelect.CENTER)

        wpilib.SmartDashboard.putData("First note", self.note_1_selection)

        self.note_2_selection = wpilib.SendableChooser()
        self.note_2_selection.addOption("Far", config.NoteSelect.FAR)
        self.note_2_selection.setDefaultOption("Mid", config.NoteSelect.MID)
        self.note_2_selection.addOption("Center", config.NoteSelect.CENTER)

        wpilib.SmartDashboard.putData("Second note", self.note_2_selection)

        # Initialize subsystems and sensors
        def init_subsystems():
            subsystems: list[Subsystem] = [
                Robot.drivetrain,
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                Robot.flywheel,
            ]

            CAN_delay(0.2)
            for subsystem in subsystems:  # noqa
                subsystem.init()
                CAN_delay(0.2)

        self.handle(init_subsystems)

        def init_sensors():

            Sensors.limelight_front.init()
            Sensors.limelight_back.init()
            Sensors.limelight_intake.init()
            Field.calculations.init()
            LEDs.leds.init()
            LEDs.leds.enable()

        self.handle(init_sensors)
        Field.calculations.tuning = True

        self.log.complete("Robot initialized")

        Robot.wrist.zero_wrist()
        # Field.odometry.disable()

    def robotPeriodic(self):
        # Leds
        if Robot.wrist.detect_note_second():
            config.active_leds = (config.LEDType.KStatic(255, 0, 0), 1, 5)
        elif Robot.intake.detect_note() or Robot.wrist.detect_note_first():
            config.active_leds = (config.LEDType.KBlink(0, 255, 0), 1, 2)
        elif Robot.intake.get_outer_current() > 0:
            config.active_leds = (config.LEDType.KRainbow(), 1, 5)
        else:
            config.active_leds = (config.LEDType.KStatic(0, 0, 255), 1, 5)

        LEDs.leds.set_LED(*config.active_leds)
        LEDs.leds.cycle()

        def get_flywheel_state():
            match states.flywheel_state:
                case states.FlywheelState.idle:
                    return 'Idle'
                case states.FlywheelState.shooting:
                    return 'Shooting'
                case states.FlywheelState.amping:
                    return 'Amping'
                case states.FlywheelState.released:
                    return 'Released'

        states_nt = self.nt.getTable('states')
        states_nt.putString('flywheel', get_flywheel_state())

        if self.team_selection.getSelected() == config.Team.BLUE:
            config.active_team = config.Team.BLUE
            constants.FieldPos.Scoring.speaker_y = 218.42 * inches_to_meters
        else:
            config.active_team = config.Team.RED
            # AT HARTFORD
            # constants.FieldPos.Scoring.speaker_y = (
            #     218.42 * inches_to_meters + 0.2
            # ) - 4 * inches_to_meters
            constants.FieldPos.Scoring.speaker_y = 218.42 * inches_to_meters

        Field.POI.setNTValues()

        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.handle(self.scheduler.run)

        self.handle(Sensors.limelight_back.update)
        self.handle(Sensors.limelight_front.update)
        self.handle(Sensors.limelight_intake.update)

        self.handle(Field.odometry.update)

        self.handle(Field.odometry.update_tables)

        self.handle(Field.calculations.update)

        self.nt.getTable("swerve").putNumberArray(
            "abs encoders", Robot.drivetrain.get_abs()
        )
        if not self.isSimulation():
            self.nt.getTable("General").putBoolean("comp bot", config.comp_bot.get())
            self.nt.getTable('General').putNumber('max vel', constants.drivetrain_max_vel)

    def teleopInit(self):
        self.log.info("Teleop initialized")
        Field.calculations.init()
        Field.odometry.set_std_tele()
        Field.odometry.enable()
        Field.odometry.enable_speaker_tags()
        Robot.wrist.zero_wrist()
        Robot.elevator.zero()
        Robot.wrist.update_wrist_pid()

        # Initialize Operator Interface
        OI.init()
        OI.map_controls()

        IT.init()
        IT.map_systems()

        self.scheduler.schedule(
            commands2.SequentialCommandGroup(
                # command.DrivetrainZero(Robot.drivetrain),
                command.DriveSwerveCustom(Robot.drivetrain),
            )
        )
        self.scheduler.schedule(
            command.DeployIntake(Robot.intake).andThen(command.IntakeIdle(Robot.intake))
        )
        # self.scheduler.schedule(
        #     command.IntakeIdle(Robot.intake)
        # )

        if Robot.wrist.note_in_feeder():
            states.flywheel_state = states.FlywheelState.shooting
        else:
            states.flywheel_state = states.FlywheelState.idle

    def teleopPeriodic(self):
        pass
        # if Robot.wrist.detect_note_first() or Robot.wrist.detect_note_second():
        #     config.active_leds = (config.LEDType.KStatic(255, 0, 0), 1, 5)
        # elif Robot.intake.get_outer_current() > 0:
        #     config.active_leds = (config.LEDType.KBlink(0, 255, 0), 1, 5)
        # else:
        #     config.active_leds = (config.LEDType.KStatic(0, 0, 255), 1, 5)
        # LEDs.leds.set_LED(*config.active_leds)
        # LEDs.leds.cycle()

    def autonomousInit(self):
        self.log.info("Autonomous initialized")
        Field.odometry.set_std_auto()
        Field.odometry.disable_speaker_tags()
        Field.calculations.init()
        Robot.drivetrain.gyro.reset_angle()
        Robot.drivetrain.n_front_left.zero()
        Robot.drivetrain.n_front_right.zero()
        Robot.drivetrain.n_back_left.zero()
        Robot.drivetrain.n_back_right.zero()

        config.first_note = self.note_1_selection.getSelected()
        config.second_note = self.note_2_selection.getSelected()

        self.auto_selection.getSelected().run()

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        # Robot.drivetrain.gyro.reset_angle(radians(180))
        # Robot.drivetrain.n_front_left.zero()
        # Robot.drivetrain.n_front_right.zero()
        # Robot.drivetrain.n_back_left.zero()
        # Robot.drivetrain.n_back_right.zero()
        # Robot.drivetrain.gyro.reset_angle(radians(180))
        #
        # new_pose = Robot.drivetrain.odometry.getPose()
        #
        # Robot.drivetrain.reset_odometry(
        #     Pose2d(
        #         new_pose.X(),
        #         new_pose.Y(),
        #         Rotation2d(180)
        #     )
        # )

        pass

    def disabledInit(self) -> None:
        self.log.info("Robot disabled")

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
