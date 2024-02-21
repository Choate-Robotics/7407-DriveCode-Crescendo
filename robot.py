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
from math import degrees, radians


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
        self.auto_selection.addOption("Five Notes", autonomous.five_note)
        self.auto_selection.addOption("Amp Three Piece", autonomous.amp_auto)

        wpilib.SmartDashboard.putData("Auto", self.auto_selection)

        self.log.info(f"Scheduler period set to {config.period} seconds")

        # Initialize subsystems and sensors
        def init_subsystems():
            subsystems: list[Subsystem] = list(
                {k: v for k, v in Robot.__dict__.items() if isinstance(v, Subsystem) and hasattr(v, 'init')}.values()
            )

            for subsystem in subsystems:
                subsystem.init()

        # try:
        #     init_subsystems()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('subsystem init', str(e))

        #     if config.DEBUG_MODE:
        #         raise e
            
        self.handle(init_subsystems)

        def init_sensors():
            sensors: list[Sensors] = list(
                {k: v for k, v in Sensors.__dict__.items() if isinstance(v, Sensors) and hasattr(v, 'init')}.values()
            )

            # for sensor in sensors:
            #     sensor.init()
            Sensors.limelight_front.init()
            Sensors.limelight_back.init()
            Sensors.limelight_intake.init()
            Field.odometry.enable()
            Field.calculations.init()

        # try:
        #     init_sensors()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('sensor init', str(e))

        #     if config.DEBUG_MODE:
        #         raise e
        
        self.handle(init_sensors)

        # Initialize Operator Interface
        OI.init()
        OI.map_controls()

        IT.init()
        IT.map_systems()

        self.log.complete("Robot initialized")

    def robotPeriodic(self):
        
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            config.active_team = config.Team.BLUE
        else:
            config.active_team = config.Team.RED

        Field.POI.setNTValues()

        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # try:
        #     self.scheduler.run()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('command scheduler', str(e))

        #     if config.DEBUG_MODE:
        #         raise e
        
        self.handle(self.scheduler.run)

        # try:
        #     Sensors.limelight_back.update()
        #     Sensors.limelight_front.update()
        #     Sensors.limelight_intake.update()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('limelight update', str(e))

        #     if config.DEBUG_MODE:
        #         raise e

        self.handle(Sensors.limelight_back.update)
        self.handle(Sensors.limelight_front.update)
        self.handle(Sensors.limelight_intake.update)

        # try:
        #     Field.odometry.update()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('odometry update', str(e))

        #     if config.DEBUG_MODE:
        #         raise e
        
        self.handle(Field.odometry.update)

        # try:
        #     Field.calculations.update()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('odometry update', str(e))

        #     if config.DEBUG_MODE:
        #         raise e
        
        self.handle(Field.calculations.update)

        self.nt.getTable('swerve').putNumberArray('abs encoders', Robot.drivetrain.get_abs())

        # print(Robot.wrist.distance_sensor.getVoltage())
        # print(Robot.intake.distance_sensor.getVoltage())
        

    def teleopInit(self):
        # self.log.info("Teleop initialized")
        Field.calculations.init()
        Robot.wrist.zero_wrist()
        Robot.elevator.zero()

        self.scheduler.schedule(commands2.SequentialCommandGroup(
            command.DrivetrainZero(Robot.drivetrain),
            command.DriveSwerveCustom(Robot.drivetrain),
        )
        )
        # self.scheduler.schedule(
        #     command.FeedIn(Robot.wrist).andThen(

        #     commands2.ParallelCommandGroup(
        #         command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAimLow),
        #         # command.AimWrist(Robot.wrist, Field.calculations),
        #         command.SetFlywheelLinearVelocity(Robot.flywheel, 27),
        #     )
        #     )
        # )
        self.scheduler.schedule(command.DeployIntake(Robot.intake).andThen(command.IntakeIdle(Robot.intake)))
        # self.scheduler.schedule(command.IntakeIdle(Robot.intake))
        self.scheduler.schedule(command.SetFlywheelLinearVelocity(Robot.flywheel, config.v0_flywheel))
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim).andThen(command.SetFlywheelLinearVelocity(Robot.flywheel, 30)))
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAimLow, Field.calculations))
        # self.scheduler.schedule(command.AimWrist(Robot.wrist, Field.calculations))
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kClimbPullUp))

    def teleopPeriodic(self):
        pass

    def autonomousInit(self):
        self.log.info("Autonomous initialized")

        Robot.drivetrain.n_front_left.zero()
        Robot.drivetrain.n_front_right.zero()
        Robot.drivetrain.n_back_left.zero()
        Robot.drivetrain.n_back_right.zero()

        self.auto_selection.getSelected().run()

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        self.log.info("Robot disabled")

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
