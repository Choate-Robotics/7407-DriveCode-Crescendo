# import math
# import time
from enum import IntEnum
from typing import Callable, List, Tuple

# import matplotlib.pyplot as plt
# import ntcore
import numpy as np

# from utils import POI
from wpimath.geometry import Pose2d, Pose3d

import config
import constants
from sensors.field_odometry import FieldOdometry
from subsystem import Elevator
from toolkit.utils.toolkit_math import NumericalIntegration, extrapolate

# from scipy.integrate import solve_ivp
# from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d

# from units.SI import seconds


class TargetVariable(IntEnum):
    """
    Enum for target variables
    """

    # TIME=-1
    X = 0
    XDOT = 1
    Z = 2
    ZDOT = 3


class Target:
    """
    Class for target to shoot at
    """

    def __init__(
        self,
        pose: Pose3d,
        velocity: float,
        goal_variable: TargetVariable,
        goal_value: float,
        end_condition: Callable[[float, List[float]], bool],
    ):
        self.pose = pose
        self.velocity = velocity
        self.end_condition = end_condition
        self.goal_variable = goal_variable
        self.goal_value = goal_value

    def get_pose2d(self) -> Pose2d:
        return self.pose.toPose2d()


class TrajectoryCalculator:
    """
    TODO: FIND DRAG COEFFICIENT!!!!!!

    Game-piece trajectory calculator that updates based on odometry and vision data.
    """

    # delta_z: float
    # speaker_z: float
    # distance_to_target: float

    def __init__(self, odometry: FieldOdometry, elevator: Elevator):
        self.odometry = odometry
        self.k = 0.5 * constants.c * constants.rho_air * constants.a
        # self.distance_to_target = 0
        # self.delta_z = 0
        self.shoot_angle = 0
        # self.base_rotation2d = Rotation2d(0)
        self.elevator = elevator

        self.numerical_integration = NumericalIntegration()

    def init(self):
        pass
        # self.speaker = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        # self.speaker_z = POI.Coordinates.Structures.Scoring.kSpeaker.getZ()

    @staticmethod
    def calculate_distance_to_target(
        robot_pose: Pose2d, target_pose: Pose3d, shooter_height: float
    ) -> Tuple[float, float]:
        delta_x = robot_pose.translation().distance(
            target_pose.toPose2d().translation()
        )
        delta_z = target_pose.translation().Z() - shooter_height
        return delta_x, delta_z

    def calculate_angle_no_air(self, target: Target) -> float:
        """
        Calculates the angle of the trajectory without air resistance.
        """
        delta_x, delta_z = self.calculate_distance_to_target(
            self.odometry.getPose(),
            target.get_pose2d(),
            self.elevator.get_length() + constants.shooter_height,
        )

        phi0 = np.arctan(delta_z / delta_x) if delta_x != 0 else 0
        result_angle = (
            0.5
            * np.arcsin(
                np.sin(phi0)
                + constants.g * delta_x * np.cos(phi0) / (config.v0_flywheel**2)
            )
            + 0.5 * phi0
        )
        return result_angle

    def update_shooter(self, target: Target) -> float:
        """
        function runs sim to calculate a final angle with air resistance considered

        :param target: target to shoot at

        :return: target angle
        """
        delta_x, delta_z = self.calculate_distance_to_target(
            self.odometry.getPose(),
            target.get_pose2d(),
            self.elevator.get_length() + constants.shooter_height,
        )
        # self.distance_to_target = (
        #     self.odometry.getPose().translation().distance(self.speaker)
        # )
        # print("distance_to_target", self.distance_to_target)

        # self.delta_z = (
        #     self.speaker_z - (self.elevator.get_length() + constants.shooter_height)
        # )
        # print("delta_z", self.delta_z)
        # print("constant.shooter_height", constants.shooter_height)
        # print("elevator.get_length()", self.elevator.get_length())
        theta_1 = self.calculate_angle_no_air(target)
        theta_2 = theta_1 + np.radians(1)
        z_1 = self.run_sim(theta_1, target)
        z_2 = self.run_sim(theta_2, target)
        z_goal_error = delta_z - z_2
        z_to_angle_conversion = (theta_2 - theta_1) / (z_2 - z_1)
        correction_angle = z_goal_error * z_to_angle_conversion
        for i in range(config.max_sim_times):
            theta_1 = theta_2
            theta_2 = theta_2 + correction_angle
            z_1 = z_2
            z_2 = self.run_sim(theta_2, target)
            z_goal_error = delta_z - z_2
            z_to_angle_conversion = (theta_2 - theta_1) / (z_2 - z_1)
            # print(z_goal_error, theta_2, self.delta_z)
            if abs(z_goal_error) < config.shooter_tol:
                self.shoot_angle = theta_2
                return theta_2
            correction_angle = z_goal_error * z_to_angle_conversion

    def update_base(self, target: Target) -> float:
        """
        updates rotation of base to face target

        :param target: target to shoot at

        :return: base target angle
        """
        target_translation = target.get_pose2d().getTranslation()
        robot_pose_2d = self.odometry.getPose()
        robot_to_speaker = target_translation - robot_pose_2d.translation()
        return robot_to_speaker.angle()

    def update(self, target: Target):
        """
        updates both shooter and base
        :return: base target angle
        """
        self.update_shooter(target)
        self.update_base(target)

    def run_sim(self, shooter_theta, target: Target) -> float:
        # def hit_target(t, u):
        #     return u[0] > self.distance_to_target

        # u0 = (
        #     0,
        #     config.v0_flywheel * np.cos(shooter_theta),
        #     0.0,
        #     config.v0_flywheel * np.sin(shooter_theta),
        # )
        u0 = (
            0,
            target.velocity * np.cos(shooter_theta),
            0.0,
            target.velocity * np.sin(shooter_theta),
        )

        t0, tf = 0, 60
        # Stop the integration when we hit the target.
        t, y = self.numerical_integration.adaptive_rk4(
            self.deriv, u0, t0, tf, 0.001, 1e-9, target.end_condition
        )

        # y[-2][0] is the penultimate x value
        # y[-2][2] is the penultimate z value
        # y[-1][0] is the final x value
        # y[-1][2] is the final z value

        goal_variable = int(target.goal_variable)
        goal_value = target.goal_value
        return extrapolate(
            goal_value, y[-2][0], y[-2][goal_variable], y[-1][0], y[-1][goal_variable]
        )

    def get_theta(self) -> float:
        """
        Returns the angle of the trajectory.
        """
        return self.shoot_angle

    def deriv(self, t, u):
        x, xdot, z, zdot = u
        speed = np.hypot(xdot, zdot)
        xdotdot = -self.k / constants.m * speed * xdot
        zdotdot = -self.k / constants.m * speed * zdot - constants.g
        return np.array([xdot, xdotdot, zdot, zdotdot])
