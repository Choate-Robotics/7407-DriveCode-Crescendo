from math import degrees, radians

import ntcore
import numpy as np
from wpimath.geometry import Rotation2d, Translation3d

import config
import constants
from sensors.field_odometry import FieldOdometry
from subsystem import Elevator
from toolkit.utils.toolkit_math import NumericalIntegration
from utils import POI


class TrajectoryCalculator:
    """
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
        self.table = ntcore.NetworkTableInstance.getDefault().getTable(
            "shot calculations"
        )
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
        if type(self.speaker) is Translation3d:
            self.speaker = self.speaker.toTranslation2d()

        self.distance_to_target = (
            self.odometry.getPose().translation().distance(self.speaker)
            - constants.shooter_offset_y
        )
        # print("distance_to_target", self.distance_to_target)

        self.delta_z = (
            self.speaker_z - self.elevator.get_length() - constants.shooter_height
        )
        theta_1 = self.calculate_angle_no_air(self.distance_to_target, self.delta_z)
        if self.use_air_resistance:
            # This is the formula for error correction for a flywheel of 22 m/s and
            # a drag coefficient of 1.28

            # y = 0.001x2 - 0.0038x + 0.0065
            theta_1 += (
                0.001 * self.distance_to_target**2
                - 0.0038 * self.distance_to_target
                + 0.0065
            )
            self.shoot_angle = theta_1
            return theta_1

        else:
            self.shoot_angle = theta_1
            return theta_1

    def update_base(self):
        """
        updates rotation of base to face target
        :return: base target angle
        """
        speaker_translation = (
            POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        )
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
        self.table.putNumber("wrist angle", degrees(self.get_theta()))
        self.table.putNumber("distance to target", self.distance_to_target)
        self.table.putNumber("bot angle", self.get_bot_theta().degrees())
        self.table.putNumber("delta z", self.delta_z)

    def get_theta(self) -> radians:
        """
        Returns the angle of the trajectory.
        """
        return self.shoot_angle

    def get_bot_theta(self) -> Rotation2d:
        """
        Returns the angle of the Robot
        """
        return self.base_rotation2d
