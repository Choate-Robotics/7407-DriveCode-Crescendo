# import math
# import time

# import matplotlib.pyplot as plt
# import ntcore
import numpy as np
from math import degrees, radians, isnan
import config, ntcore
import constants
from sensors.field_odometry import FieldOdometry
from subsystem import Elevator, Flywheel
from toolkit.utils.toolkit_math import NumericalIntegration, extrapolate
from utils import POI
from wpimath.geometry import Rotation2d, Translation3d, Translation2d, Pose2d
from units.SI import inches_to_meters


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

    def __init__(self, odometry: FieldOdometry, elevator: Elevator, flywheel: Flywheel):
        self.odometry = odometry
        self.k = 0.5 * constants.c * constants.rho_air * constants.a
        self.distance_to_target = 0
        self.delta_z = 0
        self.shoot_angle:radians = 0
        self.base_rotation2d = Rotation2d(0)
        self.elevator = elevator
        self.flywheel = flywheel
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('shot calculations')
        self.numerical_integration = NumericalIntegration()
        self.use_air_resistance = False
        self.tuning = False

    def init(self):
        self.speaker = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        self.speaker_z = POI.Coordinates.Structures.Scoring.kSpeaker.getZ()
        if self.tuning:
            self.table.putNumber('flywheel distance scalar', config.flywheel_distance_scalar)
            self.table.putNumber('flywheel minimum value', config.v0_flywheel_minimum)
            self.table.putNumber('flywheel maximum value', config.v0_flywheel_maximum)
            self.table.putNumber('shot height offset', config.shot_height_offset)

    def calculate_angle_no_air(self, distance_to_target: float, delta_z) -> radians:
        """
        Calculates the angle of the trajectory without air resistance.
        """

        vels = self.odometry.drivetrain.chassis_speeds
        
        drivetrain_angle = self.get_rotation_to_speaker()
        
        
        vels = self.odometry.drivetrain.chassis_speeds.fromRobotRelativeSpeeds(vels, drivetrain_angle)
        
        rvx, rvy, rvo = (
            vels.vx, vels.vy, vels.omega
        )
        

        # Calculate the horizontal angle without considering velocities
        phi0 = np.arctan(delta_z / distance_to_target) if distance_to_target != 0 else np.radians(90)


        # Calculate the impact of floor velocities on the trajectory

        
        # Calculate the effective velocity
        # v_effective = self.flywheel.get_velocity_linear() + rvx * np.cos(drivetrain_angle.radians()) + rvy * np.cos(drivetrain_angle.radians())
        v_effective = self.get_flywheel_speed(distance_to_target)# + rvx + rvy
        # v_effective = config.v0_flywheel

        if v_effective == 0:
            return config.Giraffe.kIdle.wrist_angle

        # Calculate the angle with floor velocities
        result_angle = (
            0.5 * np.arcsin(
                np.sin(phi0)
                + constants.g
                * distance_to_target
                * np.cos(phi0)
                / (v_effective ** 2)
            )
            + 0.5 * phi0
        )
        
        if isnan(result_angle):
            result_angle = config.Giraffe.kIdle.wrist_angle

        return result_angle
    
    def get_flywheel_speed(self, distance_to_target: float) -> float:
        if self.tuning:
            config.flywheel_distance_scalar = self.table.getNumber('flywheel distance scalar', config.flywheel_distance_scalar)
            config.v0_flywheel_minimum = self.table.getNumber('flywheel minimum value', config.v0_flywheel_minimum)
            config.v0_flywheel_maximum = self.table.getNumber('flywheel maximum value', config.v0_flywheel_maximum)
        
        return  min(config.v0_flywheel_minimum + distance_to_target * config.flywheel_distance_scalar, config.v0_flywheel_maximum)

    def update_shooter(self):
        """
        function runs sim to calculate a final angle with air resistance considered
        :return: target angle
        """
        if type(self.speaker) == Translation3d:
            self.speaker = self.speaker.toTranslation2d()

        self.distance_to_target = (
            self.odometry.getPose().translation().distance(self.speaker) - constants.shooter_offset_y
        )
        # print("distance_to_target", self.distance_to_target)

        self.delta_z = (
                self.speaker_z - self.elevator.get_length() - constants.shooter_height 
        )
        
        if self.tuning:
            config.shot_height_offset = self.table.getNumber('shot height offset', config.shot_height_offset)
            
        self.delta_z += (config.shot_height_offset * inches_to_meters)
        
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
                
    def get_rotation_to_speaker(self):
        """
        returns rotation of base to face target
        :return: base target angle
        """
        speaker_translation:Translation2d = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        robot_pose_2d = self.odometry.getPose()
        robot_to_speaker = speaker_translation - robot_pose_2d.translation()
        return robot_to_speaker.angle()

    @staticmethod
    def get_rotation_auto(robot_pose: Pose2d) -> Rotation2d:
        """
        returns rotation of base at a given pose
        meant to be used in auto
        :return: base target angle
        :rtype: Rotation2d
        """
        speaker_translation: Translation2d = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        robot_to_speaker = speaker_translation - robot_pose.translation()
        return robot_to_speaker.angle()

    def update_base(self):
        """
        updates rotation of base to score shot
        :return: base target angle
        """
        self.base_rotation2d = self.get_rotation_to_speaker()
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
        self.table.putNumber('delta z', self.delta_z)
        self.table.putNumber('flywheel speed', self.get_flywheel_speed(self.distance_to_target))
        
    def run_sim(self, shooter_theta):
        def hit_target(t, u):
            # We've hit the target if the distance to target is 0.
            return u[0] > self.distance_to_target

        u0 = (
            0,
            self.flywheel.get_velocity_linear() * np.cos(shooter_theta),
            0.0,
            self.flywheel.get_velocity_linear() * np.sin(shooter_theta),
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

    def get_theta(self) -> radians:
        """
        Returns the angle of the trajectory.
        """
        return self.shoot_angle
        # if self.use_air_resistance:
        #     return self.shoot_angle
        # else:
        #     self.distance_to_target = (
        #         self.odometry.getPose().translation().distance(self.speaker)
        #     )
        #     # print("distance_to_target", self.distance_to_target)

        #     self.delta_z = (
        #             self.speaker_z - self.elevator.get_length() - constants.shooter_height
        #     )
        #     return self.calculate_angle_no_air(self.distance_to_target, self.delta_z)

    def get_bot_theta(self) -> Rotation2d:
        """
        Returns the angle of the Robot
        """
        return self.base_rotation2d
    
    def get_distance_to_target(self) -> float:
        
        return self.distance_to_target

    def deriv(self, t, u):
        x, xdot, z, zdot = u
        speed = np.hypot(xdot, zdot)
        xdotdot = -self.k / constants.m * speed * xdot
        zdotdot = -self.k / constants.m * speed * zdot - constants.g
        return np.array([xdot, xdotdot, zdot, zdotdot])
