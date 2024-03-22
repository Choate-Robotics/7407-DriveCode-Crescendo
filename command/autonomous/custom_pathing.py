import math
import time

import ntcore, config

from toolkit.command import SubsystemCommand
from toolkit.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain
from toolkit.utils.units import radians
from toolkit.utils.toolkit_math import bounded_angle_diff, rotate_vector

from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)

from robot_systems import Sensors, Field
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians

from command.autonomous.trajectory import CustomTrajectory
from enum import Enum

from robot_systems import Field

class AngleType(Enum):

    path = 0
    calculate = 1


class FollowPathCustom(SubsystemCommand[SwerveDrivetrain]):
    """
    Follows a path using a holonomic drive controller.

    :param subsystem: The subsystem to run this command on
    :type subsystem: SwerveDrivetrain
    :param trajectory: The trajectory to follow
    :type trajectory: CustomTrajectory
    :param period: The period of the controller, defaults to 0.02
    :type period: float, optional
    :param theta_f: Desired angle in radians of the robot at the end of the trajectory
    :type theta_f: float
    """

    def __init__(
            self,
            subsystem: SwerveDrivetrain,
            trajectory: CustomTrajectory,
            period: float = config.period,
            theta_f = AngleType.path
    ):
        super().__init__(subsystem)
        self.trajectory_c: CustomTrajectory = trajectory
        self.x_controller = PIDController(8, 0, 0, period)
        self.y_controller = PIDController(8, 0, 0, period)
        constraints = TrapezoidProfileRadians.Constraints(config.drivetrain_aiming_max_angular_speed,
                                                          config.drivetrain_aiming_max_angular_accel)
        self.theta_controller = ProfiledPIDControllerRadians(
            5,
            0,
            0.08,
            constraints,
            config.period
        )
        # self.controller = HolonomicDriveController(
        #     PIDController(8, 0, 0, period),
        #     PIDController(8, 0, 0, period),
        #     self.theta_controller
        # )
        self.start_time = 0
        self.t = 0
        self.theta_i: float | None = None
        self.theta_f: float = theta_f
        self.theta_diff: float | None = None
        self.omega: float | None = None
        self.use_calculations: bool = False
        self.finished: bool = False

    def initialize(self) -> None:
        self.x_controller.reset()
        self.y_controller.reset()
        self.theta_controller.reset(Field.odometry.getPose().rotation().radians(), 0)
        self.trajectory = self.trajectory_c.generate()
        self.duration = self.trajectory.totalTime()
        self.end_pose: Pose2d = self.trajectory.states()[-1].pose
        self.start_time = time.perf_counter()
        self.theta_i = Field.odometry.getPose().rotation().radians()
        if self.theta_f == AngleType.path:
            self.theta_f = self.end_pose.rotation().radians()
        elif self.theta_f == AngleType.calculate:
            self.use_calculations = True

        # self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)
        self.finished = False
        self.theta_controller.enableContinuousInput(math.radians(-180), math.radians(180))

        traj_path = []
        for i in range(len(self.trajectory.states())):
            statepos = self.trajectory.states()[i]
            traj_path.append(
                statepos.pose.X(),
            )
            traj_path.append(
                statepos.pose.Y()
            )
            traj_path.append(
                statepos.pose.rotation().radians()
            )

        ntcore.NetworkTableInstance.getDefault().getTable('Auto').putNumberArray('trajectory', traj_path)

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time
        if self.use_calculations:
            self.theta_f = Field.calculations.get_bot_theta().radians()

        relative = self.end_pose.relativeTo(
            self.subsystem.odometry_estimator.getEstimatedPosition()
        )
        goal_reached: bool = True if (abs(bounded_angle_diff(self.theta_f, self.subsystem.odometry_estimator.getEstimatedPosition().rotation().radians())) < math.radians(1)) else False
        if (
                abs(relative.x) < 0.03
                and abs(relative.y) < 0.03
                and goal_reached
                or self.t > self.duration
        ):
            self.t = self.duration
            self.finished = True

        goal = self.trajectory.sample(self.t)
        # goal_theta = self.theta_i + (self.t/self.duration)*self.theta_diff if not goal_reached else self.theta_f
        # table = ntcore.NetworkTableInstance.getDefault().getTable('Auto')
        # table.putBoolean("goal reached", goal_reached)
        # table.putNumber("theta_i", math.degrees(self.theta_i))
        # table.putNumber("theta_f", math.degrees(self.theta_f))
        # table.putNumber("theta_diff", math.degrees(self.theta_diff))
        # table.putNumber("goal_theta", math.degrees(goal_theta))
        # table.putNumber("time", self.t)
        # table.putNumber("duration", self.duration)

        dx = self.x_controller.calculate(Field.odometry.getPose().X(), goal.pose.X())
        dy = self.y_controller.calculate(Field.odometry.getPose().Y(), goal.pose.Y())
        dtheta = self.theta_controller.calculate(Field.odometry.getPose().rotation().radians(), self.theta_f)


        self.subsystem.set_driver_centric((-dx, -dy), -dtheta)

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted: bool) -> None:
        self.subsystem.set_driver_centric((0, 0), 0)
        SmartDashboard.putString("POSE", str(self.subsystem.odometry.getPose()))
        SmartDashboard.putString(
            "POSD", str(Field.odometry.getPose().rotation().degrees())
        )

    def runsWhenDisabled(self) -> bool:
        return False



# Not working code, moving to drivetrain command
# class RotateInPlace(SubsystemCommand[SwerveDrivetrain]):
#     """
#     Rotates the robot in place.

#     :param subsystem: The subsystem to run this command on
#     :type subsystem: SwerveDrivetrain
#     :param theta_f: The final angle in radians
#     :type theta_f: radians
#     :param threshold: The angle threshold for the controller, defaults to .1
#     :type threshold: radians, optional
#     :param period: The period of the controller, defaults to 0.02
#     :type period: seconds, optional
#     """

#     def __init__(
#             self,
#             subsystem: SwerveDrivetrain,
#             theta_f: radians,
#             threshold: float = math.radians(5),
#             max_angular_vel: float | None = None,
#             period: float = 0.02,
#     ):
#         super().__init__(subsystem)

#         max_angular_vel = max_angular_vel or subsystem.max_angular_vel


#         self.controller = PIDController(
#             0,0,0
#         )

#         self.controller.setTolerance

#         self.theta_f = theta_f
#         self.threshold = threshold

#         self.theta_i: float | None = None
#         self.theta_diff: float | None = None

#     def initialize(self) -> None:
#         self.theta_i = Sensors.odometry.getPose().rotation().radians()
#         self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)

#     def execute(self) -> None:
#         current_pose = Sensors.odometry.getPose()
#         current_theta = current_pose.rotation().radians()
#         # self.theta_diff = bounded_angle_diff(current.rotation().radians(), self.theta_f)
#         error = self.controller.calculate(current_theta, self.theta_f)
#         d_theta = error * self.max_angular_vel
        
#         dx, dy = (
#             self.subsystem.axis_dx.value * (-1 if config.drivetrain_reversed else 1),
#             self.subsystem.axis_dy.value * (-1 if config.drivetrain_reversed else 1),
#         )

#         dx = curve(dx)
#         dy = curve(dy)
#         d_theta = curve(d_theta)

#         dx *= self.subsystem.max_vel
#         dy *= -self.subsystem.max_vel
#         d_theta *= self.subsystem.max_angular_vel

#         if config.driver_centric:
#             self.subsystem.set_driver_centric((dy, -dx), -d_theta)
#         elif self.driver_centric_reversed:
#             self.subsystem.set_driver_centric((-dy, dx), d_theta)
#         else:
#             self.subsystem.set_robot_centric((dy, -dx), d_theta)

#     def end(self, interrupted: bool) -> None:
#         print("ENDED ROTATE")
#         self.subsystem.set_driver_centric((0, 0), 0)

#     def isFinished(self) -> bool:
#         error = abs(Sensors.odometry.getPose().rotation().radians() - self.theta_f)
#         print("Error: ", math.degrees(error))
#         return (
#                 error
#                 < self.threshold
#         )

#     def runsWhenDisabled(self) -> bool:
#         return False
