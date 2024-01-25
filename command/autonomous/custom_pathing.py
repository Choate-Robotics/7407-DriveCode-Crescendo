import math
import time

import ntcore

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


class FollowPathCustom(SubsystemCommand[SwerveDrivetrain]):
    """
    Follows a path using a holonomic drive controller.

    :param subsystem: The subsystem to run this command on
    :type subsystem: SwerveDrivetrain
    :param trajectory: The trajectory to follow
    :type trajectory: CustomTrajectory
    :param period: The period of the controller, defaults to 0.02
    :type period: float, optional
    """

    def __init__(
            self,
            subsystem: SwerveDrivetrain,
            trajectory: CustomTrajectory,
            period: float = 0.02,
    ):
        super().__init__(subsystem)
        self.trajectory: Trajectory = trajectory.generate()
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                0.55,
                0,
                0.03,
                TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel, subsystem.max_angular_vel / 0.001  # .001
                ),
                period,
            ),
        )
        self.start_time = 0
        self.t = 0
        self.duration = self.trajectory.totalTime()
        self.theta_i: float | None = None
        self.theta_f: float | None = None
        self.theta_diff: float | None = None
        self.omega: float | None = None
        self.end_pose: Pose2d = trajectory.end_pose
        self.finished: bool = False

    def initialize(self) -> None:
        self.start_time = time.perf_counter()
        self.theta_i = Field.odometry.getPose().rotation().radians()
        self.theta_f = self.end_pose.rotation().radians()
        self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)
        self.omega = self.theta_diff / self.duration
        self.finished = False

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

        relative = self.end_pose.relativeTo(
            self.subsystem.odometry_estimator.getEstimatedPosition()
        )

        if (
                abs(relative.x) < 0.03
                and abs(relative.y) < 0.03
                and abs(relative.rotation().degrees()) < 3
                or self.t > self.duration
        ):
            self.t = self.duration
            self.finished = True

        goal = self.trajectory.sample(self.t)
        goal_theta = self.theta_i + self.omega * self.t
        speeds = self.controller.calculate(
            Field.odometry.getPose(), goal, Rotation2d(goal_theta)
        )

        vx, vy = rotate_vector(
            speeds.vx, speeds.vy, Field.odometry.getPose().rotation().radians()
        )

        self.subsystem.set_driver_centric((vx, vy), speeds.omega)

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
