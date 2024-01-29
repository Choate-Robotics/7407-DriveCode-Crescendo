# import math
# import time

# import matplotlib.pyplot as plt
# import ntcore
import numpy as np

import config
import constants
from sensors.field_odometry import FieldOdometry
from subsystem import Elevator

# from scipy.integrate import solve_ivp
# from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d

# from units.SI import seconds


class TrajectoryCalculator:
    """
    TODO: FIND DRAG COEFFICIENT!!!!!!

    Game-piece trajectory calculator that updates based on odometry and vision data.
    """

    delta_z: float
    speaker_z: float = constants.speaker_z
    distance_to_target: float

    def __init__(self, odometry: FieldOdometry, elevator: Elevator):
        self.odometry = odometry
        self.speaker = constants.speaker_location
        self.k = 0.5 * constants.c * constants.rho_air * constants.a
        self.distance_to_target = 0
        self.delta_z = 0
        self.shoot_angle = 0
        self.base_angle = 0
        self.elevator = elevator

    def calculate_angle_no_air(self, distance_to_target: float, delta_z) -> float:
        """
        Calculates the angle of the trajectory without air resistance.
        """
        phi0 = np.arctan(delta_z / distance_to_target)
        result_angle = (
            0.5
            * np.arcsin(
                np.sin(phi0)
                + constants.g
                * distance_to_target
                * np.cos(phi0)
                / (config.v0_flywheel**2)
            )
            + 0.5 * phi0
        )
        return result_angle

    def update_shooter(self):
        """
        function runs sim to calculate a final angle with air resistance considered
        :return: target angle
        """
        self.distance_to_target = (
            self.odometry.getPose().translation().distance(self.speaker)
        )
        self.delta_z = self.speaker_z - self.elevator.get_height()
        theta_1 = self.calculate_angle_no_air(self.distance_to_target, self.delta_z)
        theta_2 = theta_1 + np.radians(1)
        z_1 = self.run_sim(theta_1)
        z_2 = self.run_sim(theta_2)
        z_goal_error = self.delta_z - z_2
        z_to_angle_conversion = (theta_2 - theta_1) / (z_2 - z_1)
        correction_angle = z_goal_error * z_to_angle_conversion
        for i in range(config.max_sim_times):
            theta_1 = theta_2
            theta_2 = theta_1 + correction_angle
            z_1 = z_2
            z_2 = self.run_sim(theta_2)
            z_error = self.delta_z - z_2
            z_to_angle_conversion = (theta_2 - theta_1) / (z_2 - z_1)
            if abs(z_error) < config.shooter_tol:
                self.shoot_angle = theta_2
                return theta_2
            correction_angle = z_error * z_to_angle_conversion

    def update_base(self):
        """
        updates rotation of base to face target
        :return: base target angle
        """
        pass

    def update(self):
        """
        updates both shooter and base
        :return: base target angle
        """
        self.update_shooter()
        self.update_base()

    def run_sim(self, shooter_theta):
        def hit_target(t, u):
            # We've hit the target if the distance to target is 0.
            return self.distance_to_target - u[0]

        # u0 = (
        #     0,
        #     config.v0_flywheel * np.cos(shooter_theta),
        #     0.0,
        #     config.v0_flywheel * np.sin(shooter_theta),
        # )
        # t0, tf = 0, 60
        # Stop the integration when we hit the target.
        hit_target.terminal = True
        # We must be moving downwards (don't stop before we begin moving upwards!)
        hit_target.direction = -1
        # WILL NEED ANOTHER METHOD TO SOLVE
        # sim_solution_n = solve_ivp(
        #     self.deriv,
        #     (t0, tf),
        #     u0,
        #     method="DOP853",
        #     dense_output=True,
        #     events=(hit_target),
        # )
        # print(sim_solution)
        # t = np.linspace(0, sim_solution_n.t_events[0][0], 100)
        # sim_solution = sim_solution_n.sol(t)
        # x, z = sim_solution[0], sim_solution[2]
        z = 1
        return z

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
        return xdot, xdotdot, zdot, zdotdot
