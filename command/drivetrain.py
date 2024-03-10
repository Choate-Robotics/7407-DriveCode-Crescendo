import logging
import math
import time

import wpilib

from robot_systems import Field
from toolkit.command import SubsystemCommand

import config, constants
from subsystem import Drivetrain
from sensors import TrajectoryCalculator
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from toolkit.utils.toolkit_math import bounded_angle_diff
from math import radians
from wpimath.units import seconds


def curve_abs(x):
    curve = wpilib.SmartDashboard.getNumber('curve', 2)
    return x ** curve


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


def bound_angle(degrees: float):
    degrees = degrees % 360
    if degrees > 180:
        degrees -= 360
    return degrees


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    """
    Main drive command
    """
    driver_centric = False
    driver_centric_reversed = True

    def initialize(self) -> None:
        pass
    

    def execute(self) -> None:
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * (-1 if config.drivetrain_reversed else 1),
            self.subsystem.axis_dy.value * (-1 if config.drivetrain_reversed else 1),
            -self.subsystem.axis_rotation.value,
        )

        if abs(d_theta) < 0.11:
            d_theta = 0

        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel
        d_theta *= self.subsystem.max_angular_vel

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, -dx), -d_theta)
        elif self.driver_centric_reversed:
            self.subsystem.set_driver_centric((-dy, dx), d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DriveSwerveAim(SubsystemCommand[Drivetrain]):
    """
    Aim drivetrain at speaker based on shooter calculations
    """
    driver_centric = False
    driver_centric_reversed = True

    def __init__(self, drivetrain, target_calc: TrajectoryCalculator):
        super().__init__(drivetrain)
        self.target_calc = target_calc
        # self.theta_controller = PIDController(0.0075, 0, 0.0001, config.period)
        constraints = TrapezoidProfileRadians.Constraints(self.subsystem.max_angular_vel,
                                                          constants.drivetrain_max_angular_accel)
        self.theta_controller = ProfiledPIDControllerRadians(
            9, 0, .003,
            constraints,
            config.
            period
            )
        self.theta_controller.setTolerance(radians(1), radians(3))

    def initialize(self) -> None:
        self.theta_controller.enableContinuousInput(radians(-180), radians(180))
        self.theta_controller.reset(
            self.subsystem.odometry_estimator.getEstimatedPosition().rotation().radians(),
            self.subsystem.chassis_speeds.omega
        )


    def execute(self) -> None:
        dx, dy = (
            self.subsystem.axis_dx.value * (-1 if config.drivetrain_reversed else 1),
            self.subsystem.axis_dy.value * (-1 if config.drivetrain_reversed else 1),
        )

        target_angle = self.target_calc.get_bot_theta()
        d_theta = self.theta_controller.calculate(bound_angle(self.subsystem.odometry_estimator.getEstimatedPosition().rotation().radians()), target_angle.radians())
        
        if self.theta_controller.atSetpoint():
            self.subsystem.ready_to_shoot = True
        else:
            self.subsystem.ready_to_shoot = False

        dx = curve(dx)
        dy = curve(dy)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel
        # d_theta *= self.subsystem.max_angular_vel

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, -dx), -d_theta)
        elif self.driver_centric_reversed:
            self.subsystem.set_driver_centric((-dy, dx), d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.ready_to_shoot = False
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DrivetrainZero(SubsystemCommand[Drivetrain]):
    """
    Zeroes drivetrain
    """

    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        print("ZEROING DRIVETRAIN")
        self.subsystem.gyro.reset_angle()
        self.subsystem.n_front_left.zero()
        self.subsystem.n_front_right.zero()
        self.subsystem.n_back_left.zero()
        self.subsystem.n_back_right.zero()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        logging.info("Successfully re-zeroed swerve pods.")
        ...


class DriveSwerveHoldRotation(SubsystemCommand[Drivetrain]):
    driver_centric = False
    driver_centric_reversed = True

    def __init__(self,
                 subsystem: Drivetrain,
                 theta_f: radians,
                 threshold: float = math.radians(5),
                 max_angular_vel: float | None = None,
                 period: float = 0.02,
                 ):
        super().__init__(subsystem)
        max_angular_vel = max_angular_vel or subsystem.max_angular_vel
        self.controller = PIDController(
            9, 0, 0.003
        )

        self.controller.setTolerance(threshold)
        self.controller.enableContinuousInput(-math.pi, math.pi)
        self.controller.setTolerance(radians(3))

        self.theta_f = theta_f

        self.start_time = 0
        self.t = 0

    def initialize(self) -> None:
        self.start_time = time.perf_counter()

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time

        current_theta = self.subsystem.odometry_estimator.getEstimatedPosition().rotation().radians()
        error = self.controller.calculate(current_theta, self.theta_f)
        d_theta = error

        dx, dy = (
            self.subsystem.axis_dx.value * (-1 if config.drivetrain_reversed else 1),
            self.subsystem.axis_dy.value * (-1 if config.drivetrain_reversed else 1),

        )

        dx = curve(dx)
        dy = curve(dy)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel
        # d_theta *= self.max_angular_vel

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, -dx), -d_theta)
        elif self.driver_centric_reversed:
            self.subsystem.set_driver_centric((-dy, dx), d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return self.t > 3 or self.controller.atSetpoint()

    def runsWhenDisabled(self) -> bool:
        return False
