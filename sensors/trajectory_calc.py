from math import degrees  # noqa

import ntcore
import numpy as np
from wpimath.geometry import (  # noqa
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Translation3d,
)

import config
import constants
from sensors.field_odometry import FieldOdometry
from subsystem import Elevator
from toolkit.utils.toolkit_math import (
    NumericalIntegration,
    extrapolate,
    find_position_numpy,
)
from units.SI import radians
from utils import POI


class TrajectoryCalculator:
    """
    TODO: FIND DRAG COEFFICIENT!!!!!!

    Game-piece trajectory calculator that updates based on odometry and vision data.
    """

    def __init__(self, odometry: FieldOdometry, elevator: Elevator):
        self.odometry = odometry
        self.distance_to_target = 0
        self.delta_z = 0
        self.k = 0.5 * constants.c * constants.rho_air * constants.a
        self.shoot_angle = 0
        self.base_rotation2d = Rotation2d(0)
        self.elevator = elevator
        self.table = ntcore.NetworkTableInstance.getDefault().getTable(
            "shot calculations"
        )
        self.numerical_integration = NumericalIntegration()
        self.use_air_resistance = False
        self.use_lookup_table = False

    def init(
        self,
        set_air_resistance: bool = False,
        set_lookup_table: bool = False,
    ):
        self.use_air_resistance = set_air_resistance
        self.use_lookup_table = set_lookup_table

        self.speaker = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        self.speaker_z = POI.Coordinates.Structures.Scoring.kSpeaker.getZ()
        self.lookup_table = self.create_lookup_table()

    def create_lookup_table(self):
        """
        creates a lookup table for the angle of the trajectory
        """
        old_lookup_table = self.use_lookup_table
        self.use_lookup_table = False
        final_distance = 10
        initial_distance = 0.5
        step = 0.1
        delta_z = 1.1158003154678295
        ans_low = []
        ans_high = []
        xs = [
            initial_distance + step * i
            for i in range(int((final_distance - initial_distance) / step))
        ]
        for i in xs:
            ans_low.append(self.update_shooter(i, delta_z))
            ans_high.append(
                self.update_shooter(i, delta_z + constants.elevator_max_length)
            )
        self.use_lookup_table = old_lookup_table
        return [np.array(xs), np.array(ans_low), np.array(ans_high)]

    def calculate_angle_lookup(self, distance_to_target: float, delta_z) -> float:
        """
        calculates the angle of the trajectory using a lookup table
        """
        # lookup the distance to target in the table.
        position = find_position_numpy(self.lookup_table[0], distance_to_target)
        self.shoot_angle = extrapolate(
            distance_to_target,
            self.lookup_table[0][position - 1],
            self.lookup_table[1][position - 1],
            self.lookup_table[0][position],
            self.lookup_table[1][position],
        )
        return self.shoot_angle
        pass

    def calculate_angle_no_air(self, distance_to_target: float, delta_z) -> float:
        """
        Calculates the angle of the trajectory without air resistance.

        @return: the angle of the trajectory without air resistance
        """
        # update the distances
        # delta_x = self.target_horizontal_distance()
        # delta_z = self.target_vertical_distance()
        # print(f"odometryPose: {self.odometry.getPose()}")
        # print(f"targetPose: {self.target.get_pose2d()}")
        # print(f"delta_x: {distance_to_target}, delta_z: {delta_z}")
        # print(f"speed: {config.v0_flywheel}")
        # print(f"gravity: {constants.g}")

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

    def calculate_distance(self):
        if type(self.speaker) is Translation3d:
            self.speaker = self.speaker.toTranslation2d()

        self.distance_to_target = (
            self.odometry.getPose().translation().distance(self.speaker)
        )
        # print("distance_to_target", self.distance_to_target)

    def calculate_vertical_distance(self):
        self.delta_z = (
            self.speaker_z - self.elevator.get_length() - constants.shooter_height
        )

    def update_shooter(self, distance_to_target, delta_z) -> float:
        """
        function runs sim to calculate a final angle with air resistance considered
        :return: target angle
        """

        self.calculate_distance()
        self.calculate_vertical_distance()
        if self.use_lookup_table and self.use_air_resistance:
            return self.calculate_angle_lookup(distance_to_target, delta_z)
        else:
            # print("delta_z", self.delta_z)
            # print("velocity", config.v0_flywheel)
            # self.target.criteria.set_criteria_value(self.distance_to_target)
            theta_1 = self.calculate_angle_no_air(self.distance_to_target, self.delta_z)
            if not self.use_air_resistance:
                self.shoot_angle = theta_1
                return theta_1
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

    def update_base(self) -> Rotation2d:
        """
        updates rotation of base to face target

        :return: base target angle
        """
        speaker_translation = (
            POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        )
        robot_pose_2d = self.odometry.getPose()
        # print("robot_pose_2d: ", robot_pose_2d)

        robot_to_speaker = speaker_translation - robot_pose_2d.translation()
        # print("target", self.target.get_pose2d().translation())
        # print("robot_to_speaker", robot_to_speaker)
        self.base_rotation2d = robot_to_speaker.angle()
        return self.base_rotation2d

    def update(self):
        """
        updates both shooter and base
        saves results in class variables wrist_angle and base_angle

        """
        self.calculate_distance()
        self.calculate_vertical_distance()
        self.update_shooter(self.distance_to_target, self.delta_z)
        self.update_base()
        self.update_tables()

    def update_tables(self):
        self.table.putNumber("wrist angle", degrees(self.get_theta()))
        self.table.putNumber("distance to target", self.distance_to_target)
        self.table.putNumber("bot angle", self.get_bot_theta().degrees())

    def run_sim(self, shooter_theta: radians) -> float:
        def hit_target(t, u):
            # We've hit the target if the distance to target is 0.
            return u[0] > self.distance_to_target

        # print("distance to target", self.distance_to_target)
        # print("velocity", config.v0_flywheel)
        # print("c", constants.c)
        # print("a", constants.a)
        # print("m", constants.m)
        # print("g", constants.g)
        # print("rho_air", constants.rho_air)

        # Set the initial conditions
        u0 = (
            0,
            config.v0_flywheel * np.cos(shooter_theta),
            0.0,
            config.v0_flywheel * np.sin(shooter_theta),
        )
        # One minute should be plenty of time.
        t0, tf = 0, 60
        # Stop the integration when we hit the target.
        t, y = self.numerical_integration.adaptive_rk4(
            self.deriv, u0, t0, tf, 0.1, 1e-7, hit_target
        )

        return extrapolate(
            self.distance_to_target,
            y[-2][0],
            y[-2][2],
            y[-1][0],
            y[-1][2],
        )

    def get_theta(self) -> float:
        """
        Returns the angle of the trajectory.
        """
        return self.shoot_angle

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
