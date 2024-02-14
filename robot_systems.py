import subsystem
import sensors
import wpilib, config, constants, utils


class Robot:
    
    wrist: subsystem.Wrist = subsystem.Wrist()
    intake: subsystem.Intake = subsystem.Intake()
    elevator:subsystem.Elevator = subsystem.Elevator()
    drivetrain: subsystem.Drivetrain = subsystem.Drivetrain()
    flywheel: subsystem.Flywheel = subsystem.Flywheel()



class Pneumatics:
    pass


class Sensors:
    ...
    # limelight_front = sensors.Limelight(config.LimelightPosition.elevator_down, 'limelight-front')
    # limelight_back = sensors.Limelight(config.LimelightPosition.elevator_down, 'limelight-back')

    # odometry = sensors.FieldOdometry(Robot.drivetrain, sensors.LimelightController([limelight_front, limelight_back]))

    # limelight = sensors.Limelight(config.LimelightPosition.elevator_down)


class LEDs:
    pass


class PowerDistribution:
    pass


class Field:
    # odometry = sensors.FieldOdometry(Robot.drivetrain, sensors.LimelightController([Sensors.limelight]))
    # calculations = sensors.TrajectoryCalculator(odometry, Robot.elevator)
    POI = utils.POI()