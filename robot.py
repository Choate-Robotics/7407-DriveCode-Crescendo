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
                {k: v for k, v in Robot.__dict__.items() if isinstance(v, Subsystem) and hasattr(v, 'init')}.values()
            )

            for subsystem in subsystems:
                subsystem.init()

        try:
            init_subsystems()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable('errors').putString('subsystem init', str(e))

            if config.DEBUG_MODE:
                raise e

        def init_sensors():
            sensors: list[Sensors] = list(
                {k: v for k, v in Sensors.__dict__.items() if isinstance(v, Sensors) and hasattr(v, 'init')}.values()
            )

            # for sensor in sensors:
            #     sensor.init()
            # Sensors.limelight.init()
            # Field.odometry.enable()
            # Field.calculations.init()

        try:
            init_sensors()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable('errors').putString('sensor init', str(e))

            if config.DEBUG_MODE:
                raise e

        # Initialize Operator Interface
        OI.init()
        OI.map_controls()

        IT.init()
        IT.map_systems()

        self.log.complete("Robot initialized")

    def robotPeriodic(self):

        Field.POI.setNTValues()

        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        try:
            self.scheduler.run()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable('errors').putString('command scheduler', str(e))

            if config.DEBUG_MODE:
                raise e

        SmartDashboard.putNumber("Front right current abs position", Robot.drivetrain.n_front_right.get_abs())
        SmartDashboard.putNumber("Front left current abs position", Robot.drivetrain.n_front_left.get_abs())
        SmartDashboard.putNumber("Back right current abs position", Robot.drivetrain.n_back_right.get_abs())
        SmartDashboard.putNumber("Back left current abs position", Robot.drivetrain.n_back_left.get_abs())

        SmartDashboard.putNumber("Front right abs position", config.front_right_encoder_zeroed_pos)
        SmartDashboard.putNumber("Front left abs position", config.front_left_encoder_zeroed_pos)
        SmartDashboard.putNumber("Back right abs position", config.back_right_encoder_zeroed_pos)
        SmartDashboard.putNumber("Back left abs position", config.back_left_encoder_zeroed_pos)

        # Turn motor angle
        SmartDashboard.putNumber("Front right angle", Robot.drivetrain.n_front_right.get_turn_motor_angle())
        SmartDashboard.putNumber("Front left angle", Robot.drivetrain.n_front_left.get_turn_motor_angle())
        SmartDashboard.putNumber("Back right angle", Robot.drivetrain.n_back_right.get_turn_motor_angle())
        SmartDashboard.putNumber("Back left angle", Robot.drivetrain.n_back_left.get_turn_motor_angle())

        # Motor rotation position
        SmartDashboard.putNumber("Front right sensor position",
                                 Robot.drivetrain.n_front_right.m_turn.get_sensor_position())
        SmartDashboard.putNumber("Front left sensor position",
                                 Robot.drivetrain.n_front_left.m_turn.get_sensor_position())
        SmartDashboard.putNumber("Back right sensor position",
                                 Robot.drivetrain.n_back_right.m_turn.get_sensor_position())
        SmartDashboard.putNumber("Back left sensor position",
                                 Robot.drivetrain.n_back_left.m_turn.get_sensor_position())
        # Gyro angle
        SmartDashboard.putNumber("Gyro Angle", Robot.drivetrain.get_heading().degrees())


        # try:

            # Sensors.limelight_back.update()
            # Sensors.limelight.update()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('limelight update', str(e))
        #
        #     if config.DEBUG_MODE:
        #         raise e

        # try:
        #     Field.odometry.update()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('odometry update', str(e))
        #
        #     if config.DEBUG_MODE:
        #         raise e

        # try:
        #     Field.calculations.update()
        # except Exception as e:
        #     self.log.error(str(e))
        #     self.nt.getTable('errors').putString('odometry update', str(e))
        #
        #     if config.DEBUG_MODE:
        #         raise e

    def teleopInit(self):
        # self.log.info("Teleop initialized")
        self.scheduler.schedule(commands2.SequentialCommandGroup(
            command.DrivetrainZero(Robot.drivetrain),
            command.DriveSwerveCustom(Robot.drivetrain)
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
