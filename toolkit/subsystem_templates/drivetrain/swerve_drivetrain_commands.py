import time

from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import Rotation2d
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians

from toolkit.command import SubsystemCommand
from toolkit.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain
from toolkit.utils.toolkit_math import bounded_angle_diff, rotate_vector


class DriveSwerve(SubsystemCommand[SwerveDrivetrain]):
    """
    Drive the robot using a swerve drive controller.
    """

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        """
        Execute the command. Can be overridden for a custom swerve drive.
        """
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value,
            self.subsystem.axis_dy.value,
            self.subsystem.axis_rotation.value,
        )

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel

        self.subsystem.set_driver_centric(
            (dx, dy), -d_theta * self.subsystem.max_angular_vel
        )

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class FollowPath(SubsystemCommand[SwerveDrivetrain]):
    """
    Follow a given wpimath trajectory using a swerve drive controller.
    """

    def __init__(
        self, subsystem: SwerveDrivetrain, trajectory: Trajectory, period: float = 0.02
    ):
        super().__init__(subsystem)
        self.trajectory = trajectory
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0),
            PIDController(1, 0, 0),
            ProfiledPIDControllerRadians(
                8,
                0,
                0,
                TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel, subsystem.max_angular_vel / 0.01
                ),
                period,
            ),
        )
        self.start_time = 0
        self.t = 0
        self.duration = trajectory.totalTime()
        self.theta_i = trajectory.initialPose().rotation().radians()
        self.theta_f = trajectory.sample(self.duration).pose.rotation().radians()
        self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)
        self.omega = self.theta_diff / self.duration

    def initialize(self) -> None:
        self.start_time = time.perf_counter()

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time
        if self.t > self.duration:
            self.t = self.duration
        goal = self.trajectory.sample(self.t)
        goal_theta = self.theta_i + self.omega * self.t
        speeds = self.controller.calculate(
            self.subsystem.odometry.getPose(), goal, Rotation2d(goal_theta)
        )
        vx, vy = rotate_vector(
            speeds.vx, speeds.vy, self.subsystem.odometry.getPose().rotation().radians()
        )
        self.subsystem.set_driver_centric((vx, vy), speeds.omega)

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return self.t > self.trajectory.totalTime()

    def runsWhenDisabled(self) -> bool:
        return False
