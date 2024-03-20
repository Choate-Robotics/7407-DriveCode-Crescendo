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
        self.auto_selection.setDefaultOption("Two Notes", autonomous.two_note)
        self.auto_selection.addOption("Midline Auto", autonomous.mid_notes)
        self.auto_selection.addOption("Four Notes", autonomous.four_note)
        self.auto_selection.addOption("Left Four Notes", autonomous.left_four_note)
        self.auto_selection.addOption("Right Three Notes", autonomous.right_three_note)
        self.auto_selection.addOption("Five Notes", autonomous.five_note)
        self.auto_selection.addOption("Amp Three Piece", autonomous.amp_auto)
        self.auto_selection.addOption("Shoot Note", autonomous.aim_shoot_auto)
        self.auto_selection.addOption("Four Note Middle", autonomous.four_note_middle)

        wpilib.SmartDashboard.putData("Auto", self.auto_selection)

        self.team_selection = wpilib.SendableChooser()
        self.team_selection.setDefaultOption("Blue", config.Team.BLUE)
        self.team_selection.addOption("Red", config.Team.RED)

        wpilib.SmartDashboard.putData("Team", self.team_selection)

        self.log.info(f"Scheduler period set to {config.period} seconds")

        # Initialize subsystems and sensors
        def init_subsystems():
            subsystems: list[Subsystem] = [
                Robot.drivetrain,
                Robot.elevator,
                Robot.wrist,
                Robot.intake,
                Robot.flywheel,
            ]

            time.sleep(0.2)
            for subsystem in subsystems:  # noqa
                subsystem.init()
                time.sleep(0.2)

        # try:
        #     init_subsystems()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('subsystem init', str(e))

        #     if config.DEBUG_MODE:
        #         raise e

        self.handle(init_subsystems)

        def init_sensors():
            sensors: list[Sensors] = list(  # noqa
                {
                    k: v
                    for k, v in Sensors.__dict__.items()
                    if isinstance(v, Sensors) and hasattr(v, "init")
                }.values()
            )

            # for sensor in sensors:
            #     sensor.init()
            Sensors.limelight_front.init()
            Sensors.limelight_back.init()
            Sensors.limelight_intake.init()
            Field.calculations.init()

        # try:
        #     init_sensors()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('sensor init', str(e))

        #     if config.DEBUG_MODE:
        #         raise e

        self.handle(init_sensors)
        Field.calculations.tuning = True

        self.log.complete("Robot initialized")

        Robot.wrist.zero_wrist()

    def robotPeriodic(self):
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

        self.handle(Field.calculations.update)

        self.nt.getTable("swerve").putNumberArray(
            "abs encoders", Robot.drivetrain.get_abs()
        )
        if not self.isSimulation():
            self.nt.getTable("swerve").putBoolean("comp bot", config.comp_bot.get())

    def teleopInit(self):
        self.log.info("Teleop initialized")
        Field.calculations.init()
        Field.odometry.set_std_tele()
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
                command.DrivetrainZero(Robot.drivetrain),
                command.DriveSwerveCustom(Robot.drivetrain),
            )
        )
        self.scheduler.schedule(
            command.DeployIntake(Robot.intake).andThen(command.IntakeIdle(Robot.intake))
        )
        # self.scheduler.schedule(command.IntakeIdle(Robot.intake))
        self.scheduler.schedule(
            command.SetFlywheelLinearVelocity(Robot.flywheel, config.idle_flywheel)
        )
        # self.scheduler.schedule(commands2.InstantCommand(lambda: Robot.flywheel.motor_1.set_raw_output(1)))
        # self.scheduler.schedule(command.SetWrist(Robot.wrist, radians(0)).andThen(commands2.WaitCommand(3))
        # .andThen(command.SetWrist(Robot.wrist, radians(55))))
        # self.scheduler.schedule(command.SetWrist(Robot.wrist, radians(-20)))

        # self.scheduler.schedule()
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim)
        # .andThen(command.SetFlywheelLinearVelocity(Robot.flywheel, 30)))
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist,
        # config.Giraffe.kAimLow, Field.calculations))
        # self.scheduler.schedule(command.AimWrist(Robot.wrist, Field.calculations))
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kClimbPullUp))
        # self.scheduler.schedule(command.SetElevator(Robot.elevator, constants.elevator_max_length)
        # .andThen(command.SetElevator(Robot.elevator, 0)))

    def teleopPeriodic(self):
        pass

    def autonomousInit(self):
        self.log.info("Autonomous initialized")
        Field.odometry.set_std_auto()
        Field.calculations.init()
        Robot.drivetrain.n_front_left.zero()
        Robot.drivetrain.n_front_right.zero()
        Robot.drivetrain.n_back_left.zero()
        Robot.drivetrain.n_back_right.zero()

        self.auto_selection.getSelected().run()

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        # Robot.drivetrain.gyro.reset_angle(radians(-180))
        # Robot.drivetrain.n_front_left.zero()
        # Robot.drivetrain.n_front_right.zero()
        # Robot.drivetrain.n_back_left.zero()
        # Robot.drivetrain.n_back_right.zero()
        ...

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
