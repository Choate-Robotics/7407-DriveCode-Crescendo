import subsystem
import sensors
import wpilib, config, constants, utils


class Robot:
    wrist: subsystem.Wrist = subsystem.Wrist()
    intake: subsystem.Intake = subsystem.Intake()
    elevator: subsystem.Elevator = subsystem.Elevator()
    drivetrain: subsystem.Drivetrain = subsystem.Drivetrain()
    flywheel: subsystem.Flywheel = subsystem.Flywheel()


class Pneumatics:
    pass


class Sensors:
    limelight_intake = sensors.Limelight(config.LimelightPosition.init_elevator_front, 'limelight-i')
    limelight_back = sensors.Limelight(config.LimelightPosition.init_elevator_back, 'limelight-b')

    limelight_front = sensors.Limelight(config.LimelightPosition.init_elevator_front)


class LEDs:
    pass


class PowerDistribution:
    pass


class Field:
    odometry = sensors.FieldOdometry(Robot.drivetrain,
                                     sensors.LimelightController([Sensors.limelight_front, Sensors.limelight_back]))
    calculations = sensors.TrajectoryCalculator(odometry, Robot.elevator)
    POI = utils.POI()
