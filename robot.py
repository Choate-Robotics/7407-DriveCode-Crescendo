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
from wpilib import SmartDashboard, DigitalInput
from math import degrees, radians, pi


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
        if not self.isSimulation():
            self.nt.getTable('swerve').putBoolean('comp bot', config.comp_bot.get())
            
        self.nt.getTable('swerve').putNumber('abs front right', Robot.drivetrain.get_abs()[1])
        self.nt.getTable('swerve').putNumber('front right rotation', Robot.drivetrain.n_front_right.get_turn_motor_angle() / (2 * pi))
        self.nt.getTable('swerve').putNumber('front right rotation error', (Robot.drivetrain.n_front_right.get_turn_motor_angle() / (2 * pi)) - Robot.drivetrain.get_abs()[1])
        
        
        # print(config.elevator_zeroed_pos)
        # print(Robot.wrist.distance_sensor.getVoltage())
        # print(Robot.intake.distance_sensor.getVoltage())
        # print(DigitalInput(0).get())
        
        self.nt.getTable('pdh').putNumber('ch 1 current', PowerDistribution.pd.getCurrent(1))
        self.nt.getTable('pdh').putNumber('ch 0 current', PowerDistribution.pd.getCurrent(0))
        

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
        # self.scheduler.schedule(command.DeployIntake(Robot.intake).andThen(command.IntakeIdle(Robot.intake)))
        # self.scheduler.schedule(command.IntakeIdle(Robot.intake))
        # self.scheduler.schedule(command.SetFlywheelLinearVelocity(Robot.flywheel, config.v0_flywheel))
        # self.scheduler.schedule(commands2.InstantCommand(lambda: Robot.flywheel.motor_1.set_raw_output(1)))
        # self.scheduler.schedule(command.SetWrist(Robot.wrist, radians(0)).andThen(commands2.WaitCommand(3)).andThen(command.SetWrist(Robot.wrist, radians(55))))
        # self.scheduler.schedule(command.SetWrist(Robot.wrist, radians(0)))
        
        # self.scheduler.schedule()
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAim).andThen(command.SetFlywheelLinearVelocity(Robot.flywheel, 30)))
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kAimLow, Field.calculations))
        # self.scheduler.schedule(command.AimWrist(Robot.wrist, Field.calculations))
        # self.scheduler.schedule(command.Giraffe(Robot.elevator, Robot.wrist, config.Giraffe.kClimbPullUp))
        # self.scheduler.schedule(command.SetElevator(Robot.elevator, constants.elevator_max_length).andThen(command.SetElevator(Robot.elevator, 0)))

    def teleopPeriodic(self):
        ...
        # print(Robot.elevator.get_length_total_height())
        # print(degrees(Robot.wrist.get_wrist_abs_angle() ))
        # print(degrees(Robot.wrist.get_wrist_angle()))
        # print(Robot.wrist.wrist_motor.get_sensor_position())
        # print(Robot.flywheel.get_velocity_linear())

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
