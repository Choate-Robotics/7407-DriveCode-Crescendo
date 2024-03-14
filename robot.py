import commands2
from toolkit.subsystem import Subsystem
import ntcore
import wpilib
import command
import config
import constants
from robot_systems import Robot, Pneumatics, Sensors, LEDs, PowerDistribution, Field
import sensors
import subsystem
import utils
from oi.OI import OI
from oi.IT import IT
from wpilib import SmartDashboard
import autonomous
import math
from math import degrees, radians, pi
import time
from wpimath.geometry import Rotation2d, Pose2d

from units.SI import inches_to_meters


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.log = utils.LocalLogger("Robot")
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.scheduler = commands2.CommandScheduler.getInstance()

    def handle(self, func, *args, **kwargs):
        try:
            func(*args, **kwargs)
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable('errors').putString(func.__name__, str(e))

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
            for subsystem in subsystems:
                subsystem.init()
                time.sleep(0.2)


        self.handle(init_subsystems)

        def init_sensors():

            Sensors.limelight_front.init()
            Sensors.limelight_back.init()
            Sensors.limelight_intake.init()
            Field.calculations.init()


        self.handle(init_sensors)

        self.log.complete("Robot initialized")

        Robot.wrist.zero_wrist()

    def robotPeriodic(self):

        if self.team_selection.getSelected() == config.Team.BLUE:
            config.active_team = config.Team.BLUE
            constants.FieldPos.Scoring.speaker_y = 218.42 * inches_to_meters
        else:
            config.active_team = config.Team.RED
            constants.FieldPos.Scoring.speaker_y = (218.42 * inches_to_meters + 0.2) - 4 * inches_to_meters

        Field.POI.setNTValues()

        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.handle(self.scheduler.run)

        self.handle(Sensors.limelight_back.update)
        self.handle(Sensors.limelight_front.update)
        self.handle(Sensors.limelight_intake.update)

        self.handle(Field.odometry.update)

        self.handle(Field.calculations.update)

        self.nt.getTable('swerve').putNumberArray('abs encoders', Robot.drivetrain.get_abs())
        if not self.isSimulation():
            self.nt.getTable('swerve').putBoolean('comp bot', config.comp_bot.get())

        self.nt.getTable('swerve').putNumber('abs front right', Robot.drivetrain.get_abs()[1])
        self.nt.getTable('swerve').putNumber('front right rotation',
                                             Robot.drivetrain.n_front_right.get_turn_motor_angle() / (2 * pi))
        self.nt.getTable('swerve').putNumber('front right rotation error',
                                             (Robot.drivetrain.n_front_right.get_turn_motor_angle() / (2 * pi)) -
                                             Robot.drivetrain.get_abs()[1])

        # print(config.elevator_zeroed_pos)
        # print(Robot.wrist.distance_sensor.getVoltage())
        # print(Robot.intake.distance_sensor.getVoltage())
        # print(DigitalInput(0).get())

        self.nt.getTable('pdh').putNumber('ch 1 current', PowerDistribution.pd.getCurrent(1))
        self.nt.getTable('pdh').putNumber('ch 0 current', PowerDistribution.pd.getCurrent(0))
        # print(config.WRIST_CONFIG.k_P)

    def teleopInit(self):
        self.log.info("Teleop initialized")
        Field.calculations.init()
        Field.odometry.set_std_tele()
        Robot.wrist.zero_wrist()
        Robot.elevator.zero()

        # Initialize Operator Interface
        OI.init()
        OI.map_controls()

        IT.init()
        IT.map_systems()

        self.scheduler.schedule(commands2.SequentialCommandGroup(
            # command.DrivetrainZero(Robot.drivetrain),
            command.DriveSwerveCustom(Robot.drivetrain),
        )
        )
        self.scheduler.schedule(command.DeployIntake(Robot.intake).andThen(command.IntakeIdle(Robot.intake)))
        self.scheduler.schedule(command.SetFlywheelLinearVelocity(Robot.flywheel, config.idle_flywheel))

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
