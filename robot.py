import commands2
import ntcore
import wpilib

import command
import config

# import constants
# import sensors
# import subsystem
import utils
from oi.IT import IT
from oi.OI import OI
from robot_systems import Field, Robot, Sensors
from toolkit.subsystem import Subsystem

# from wpilib import SmartDashboard


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.log = utils.LocalLogger("Robot")
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.scheduler = commands2.CommandScheduler.getInstance()

    def robotInit(self):
        self.log._robot_log_setup()

        if config.DEBUG_MODE:
            self.log.setup("WARNING: DEBUG MODE IS ENABLED")

        self.scheduler.setPeriod(config.period)

        self.log.info(f"Scheduler period set to {config.period} seconds")

        # Initialize subsystems and sensors
        def init_subsystems():
            subsystems: list[Subsystem] = list(
                {
                    k: v
                    for k, v in Robot.__dict__.items()
                    if isinstance(v, Subsystem) and hasattr(v, "init")
                }.values()
            )

            for subsystem in subsystems:
                subsystem.init()

        try:
            init_subsystems()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString("subsystem init", str(e))

            if config.DEBUG_MODE:
                raise e

        def init_sensors():
            sensors: list[Sensors] = list(
                {
                    k: v
                    for k, v in Sensors.__dict__.items()
                    if isinstance(v, Sensors) and hasattr(v, "init")
                }.values()
            )

            for sensor in sensors:
                sensor.init()
            Sensors.limelight.init()
            Field.odometry.enable()
            Field.speaker_calculations.init()

        try:
            init_sensors()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString("sensor init", str(e))

            if config.DEBUG_MODE:
                raise e

        # Initialize Operator Interface
        OI.init()
        OI.map_controls()

        IT.init()
        IT.map_systems()

        self.log.complete("Robot initialized")

        # Initialize Operator Interface
        OI.init()
        OI.map_controls()

        IT.init()
        IT.map_systems()

    def robotPeriodic(self):
        Field.POI.setNTValues()

        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        try:
            self.scheduler.run()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString("command scheduler", str(e))

            if config.DEBUG_MODE:
                raise e

        try:
            # Sensors.limelight_back.update()
            Sensors.limelight.update()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString("limelight update", str(e))

            if config.DEBUG_MODE:
                raise e

        try:
            Field.odometry.update()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString("odometry update", str(e))

            if config.DEBUG_MODE:
                raise e

        try:
            Field.speaker_calculations.update()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString("odometry update", str(e))

            if config.DEBUG_MODE:
                raise e

    def teleopInit(self):
        # self.log.info("Teleop initialized")
        self.scheduler.schedule(
            commands2.SequentialCommandGroup(
                command.DrivetrainZero(Robot.drivetrain),
                command.DriveSwerveCustom(Robot.drivetrain),
            )
        )

    def teleopPeriodic(self):
        pass

    def autonomousInit(self):
        self.log.info("Autonomous initialized")

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        self.log.info("Robot disabled")

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
