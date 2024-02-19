# import math
# import time

# import matplotlib.pyplot as plt
# import ntcore
import numpy as np
from math import degrees, radians
import config, ntcore
import constants
from sensors.field_odometry import FieldOdometry
from subsystem import Elevator
from toolkit.utils.toolkit_math import NumericalIntegration, extrapolate
from utils import POI
from wpimath.geometry import Rotation2d, Translation3d, Translation2d


# from scipy.integrate import solve_ivp
# from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d

# from units.SI import seconds


class TrajectoryCalculator:
    """
    TODO: FIND DRAG COEFFICIENT!!!!!!

    Game-piece trajectory calculator that updates based on odometry and vision data.
    """

    delta_z: float
    speaker_z: float
    distance_to_target: float

    def __init__(self, odometry: FieldOdometry, elevator: Elevator):
        self.odometry = odometry
        self.k = 0.5 * constants.c * constants.rho_air * constants.a
        self.distance_to_target = 0
        self.delta_z = 0
        self.shoot_angle = 0
        self.base_rotation2d = Rotation2d(0)
        self.elevator = elevator
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('shot calculations')
        self.numerical_integration = NumericalIntegration()
        self.use_air_resistance = False

    def init(self):
        self.speaker = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        self.speaker_z = POI.Coordinates.Structures.Scoring.kSpeaker.getZ()

    def calculate_angle_no_air(self, distance_to_target: float, delta_z) -> float:
        """
        Calculates the angle of the trajectory without air resistance.
        """

        phi0 = np.arctan(delta_z / distance_to_target) if distance_to_target != 0 else 0
        result_angle = (
                0.5
                * np.arcsin(
            np.sin(phi0)
            + (constants.g
               * distance_to_target
               * np.cos(phi0)
               / (config.v0_flywheel ** 2))
        )
                + 0.5 * phi0
        )
        return result_angle

    def update_shooter(self):
        """
        function runs sim to calculate a final angle with air resistance considered
        :return: target angle
        """
        if type(self.speaker) == Translation3d:
            self.speaker = self.speaker.toTranslation2d()

        self.distance_to_target = (
            self.odometry.getPose().translation().distance(self.speaker)
        )
        # print("distance_to_target", self.distance_to_target)

        self.delta_z = (
                self.speaker_z - self.elevator.get_length() + constants.shooter_height
        )
        # print("delta_z", self.delta_z)
        # print("constant.shooter_height", constants.shooter_height)
        # print("elevator.get_length()", self.elevator.get_length())
        theta_1 = self.calculate_angle_no_air(self.distance_to_target, self.delta_z)
        if not self.use_air_resistance:
            self.shoot_angle = theta_1
            return theta_1
        else:
            theta_2 = theta_1 + np.radians(1)
            z_1 = self.run_sim(theta_1)
            z_2 = self.run_sim(theta_2)
            z_goal_error = self.delta_z - z_2
            z_to_angle_conversion = (theta_2 - theta_1) / (z_2 - z_1)
            correction_angle = z_goal_error * z_to_angle_conversion
            for i in range(config.max_sim_times):
                theta_1 = theta_2
                theta_2 = theta_2 + correction_angle
                z_1 = z_2
                z_2 = self.run_sim(theta_2)
                z_goal_error = self.delta_z - z_2
                z_to_angle_conversion = (theta_2 - theta_1) / (z_2 - z_1)
                # print(z_goal_error, theta_2, self.delta_z)
                if abs(z_goal_error) < config.shooter_tol:
                    self.shoot_angle = theta_2
                    return theta_2
                correction_angle = z_goal_error * z_to_angle_conversion

    def update_base(self):
        """
        updates rotation of base to face target
        :return: base target angle
        """
        speaker_translation = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        robot_pose_2d = self.odometry.getPose()
        robot_to_speaker = speaker_translation - robot_pose_2d.translation()
        self.base_rotation2d = robot_to_speaker.angle()
        return self.base_rotation2d

    def update(self):
        """
        updates both shooter and base
        :return: base target angle
        """

        self.update_shooter()
        self.update_base()
        self.update_tables()

    def update_tables(self):
        self.table.putNumber('wrist angle', degrees(self.get_theta()))
        self.table.putNumber('distance to target', self.distance_to_target)
        self.table.putNumber('bot angle', self.get_bot_theta().degrees())

    def run_sim(self, shooter_theta):
        def hit_target(t, u):
            # We've hit the target if the distance to target is 0.
            return u[0] > self.distance_to_target

        u0 = (
            0,
            config.v0_flywheel * np.cos(shooter_theta),
            0.0,
            config.v0_flywheel * np.sin(shooter_theta),
        )
        t0, tf = 0, 60
        # Stop the integration when we hit the target.
        t, y = self.numerical_integration.adaptive_rk4(
            self.deriv, u0, t0, tf, 0.001, 1e-9, hit_target
        )

        # y[-2][0] is the penultimate x value
        # y[-2][2] is the penultimate z value
        # y[-1][0] is the final x value
        # y[-1][2] is the final z value
        return extrapolate(
            self.distance_to_target, y[-2][0], y[-2][2], y[-1][0], y[-1][2]
        )

    def get_theta(self) -> float:
        """
        Returns the angle of the trajectory.
        """
        if self.use_air_resistance:
            return self.shoot_angle
        else:
            self.distance_to_target = (
                self.odometry.getPose().translation().distance(self.speaker)
            )
            # print("distance_to_target", self.distance_to_target)

            self.delta_z = (
                    self.speaker_z - self.elevator.get_length() + constants.elevator_bottom_total_height
            )
            return self.calculate_angle_no_air(self.distance_to_target, self.delta_z)

    def get_bot_theta(self) -> Rotation2d:
        """
        Returns the angle of the Robot
        """
        return self.base_rotation2d

    def deriv(self, t, u):
        x, xdot, z, zdot = u
        speed = np.hypot(xdot, zdot)
        xdotdot = -self.k / constants.m * speed * xdot
        zdotdot = -self.k / constants.m * speed * zdot - constants.g
        return np.array([xdot, xdotdot, zdot, zdotdot])
