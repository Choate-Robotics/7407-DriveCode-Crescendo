import subsystem
import sensors
import wpilib, config, constants, utils


class Robot:
    # intake = subsystem.Intake()
    # elevator = subsystem.Elevator()
    drivetrain = subsystem.Drivetrain()


class Pneumatics:
    pass


class Sensors:
    # limelight_front = sensors.Limelight(config.LimelightPosition.elevator_down, 'limelight-front')
    limelight_back = sensors.Limelight(config.LimelightPosition.elevator_down, 'limelight-b')

    # odometry = sensors.FieldOdometry(Robot.drivetrain, sensors.LimelightController([limelight_front, limelight_back]))

    limelight = sensors.Limelight(config.LimelightPosition.elevator_down)


class LEDs:
    pass


class PowerDistribution:
    pass


class Field:
    odometry = sensors.FieldOdometry(Robot.drivetrain, sensors.LimelightController([Sensors.limelight, Sensors.limelight_back]))
    POI = utils.POI()