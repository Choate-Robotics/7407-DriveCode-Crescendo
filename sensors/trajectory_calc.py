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
        self.distance_to_feed_zone = 0
        self.delta_z = 0
        self.shoot_angle:radians = 0
        self.feed_angle:radians = 0
        self.base_rotation2d = Rotation2d(0)
        self.feed_rotation2d = Rotation2d(0)
        self.bot_shot_tolerance:radians = radians(1)
        self.t_total = 0
        self.elevator = elevator
        self.flywheel = flywheel
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('shot calculations')
        self.numerical_integration = NumericalIntegration()
        self.use_air_resistance = False
        self.tuning = False

    def init(self):
        self.speaker = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        self.speaker_z = POI.Coordinates.Structures.Scoring.kSpeaker.getZ()
        self.feed_zone = POI.Coordinates.Structures.Scoring.kFeed.getTranslation()
        self.feed_zone_z = POI.Coordinates.Structures.Scoring.kFeed.getZ()
        self.feed_zone_midline = POI.Coordinates.Structures.Scoring.kFeedMidline.getTranslation()
        self.feed_zone_midline_z = POI.Coordinates.Structures.Scoring.kFeedMidline.getZ()
        self.feed_static = POI.Coordinates.Structures.Scoring.kFeedStatic.getTranslation()
        self.feed_static_z = POI.Coordinates.Structures.Scoring.kFeedStatic.getZ()
        if self.tuning:
            self.table.putNumber('flywheel distance scalar', config.flywheel_distance_scalar)
            self.table.putNumber('flywheel minimum value', config.v0_flywheel_minimum)
            self.table.putNumber('flywheel maximum value', config.v0_flywheel_maximum)
            self.table.putNumber('shot height offset', config.shot_height_offset)
            self.table.putNumber('height offset scalar', config.shot_height_offset_scalar)
            self.table.putNumber('shot angle offset', config.shot_angle_offset)

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
    
    def calculate_lob_speeds(self, distance_to_target, target_height, z_init=0) -> tuple[float, float]:
        
        vy = np.sqrt(2 * constants.g * (target_height - z_init))
        
        time = vy * (2/constants.g)
        
        vx = distance_to_target / time 
        
        return vy, vx
    
    def calculate_angle_feed_zone(self, distance_to_target: float, z_init: float) -> radians:
        
        vy, vx = self.calculate_lob_speeds(distance_to_target, config.feed_shot_target_height, z_init)
        
        theta = np.arctan(vy / vx) if vx != 0 else np.radians(90)
        
        if theta > np.radians(60):
            theta = np.radians(55)
        
        return theta
    
    def get_flywheel_speed_feed(self, distance_to_target: float) -> float:
        
        
        vy, vx = self.calculate_lob_speeds(distance_to_target, config.feed_shot_target_height, self.get_shooter_height())
        
        v_total = np.sqrt(vy ** 2 + vx **2) + 4
        # print(vx, vy, v_total)
        
        return  min(v_total, config.v0_flywheel_maximum)
    
    def get_shooter_height(self) -> float:
        return constants.shooter_height + self.elevator.get_length()

    def get_flywheel_speed(self, distance_to_target:float) -> float:
        if self.tuning:
            config.flywheel_distance_scalar = self.table.getNumber('flywheel distance scalar', config.flywheel_distance_scalar)
            config.v0_flywheel_minimum = self.table.getNumber('flywheel minimum value', config.v0_flywheel_minimum)
            config.v0_flywheel_maximum = self.table.getNumber('flywheel maximum value', config.v0_flywheel_maximum)
        
        return  min(config.v0_flywheel_minimum + distance_to_target * config.flywheel_distance_scalar, config.v0_flywheel_maximum)

    def get_flywheel_shot_tolerance(self, distance_to_target:float) -> float:
        
        slope = (config.flywheel_min_shot_tolerance - config.flywheel_max_shot_tolerance) / (config.flywheel_min_shot_tolerance_distance - config.flywheel_max_shot_tolerance_distance)
        if distance_to_target < config.flywheel_max_shot_tolerance_distance:
            return config.flywheel_max_shot_tolerance
        if distance_to_target > config.flywheel_min_shot_tolerance_distance:
            return config.flywheel_min_shot_tolerance
        return distance_to_target * slope + config.flywheel_max_shot_tolerance

    def get_height_offset(self, distance_to_target: float) -> float:
        if self.tuning:
            config.shot_height_offset_scalar = self.table.getNumber('height offset scalar', config.shot_height_offset_scalar)

        return distance_to_target * config.shot_height_offset_scalar
    
    
    def update_shooter(self):
        """
        function runs sim to calculate a final angle with air resistance considered
        :return: target angle
        """
        if type(self.speaker) == Translation3d:
            self.speaker = self.speaker.toTranslation2d()
            
        if type(self.feed_zone) == Translation3d:
            self.feed_zone = self.feed_zone.toTranslation2d()
            
        if type(self.feed_zone_midline) == Translation3d:
            self.feed_zone_midline = self.feed_zone.toTranslation2d()

        self.distance_to_target = (
            self.odometry.getPose().translation().distance(self.speaker) - constants.shooter_offset_y
        )
        
        active_feed_zone = self.feed_zone
        
        pose_x = self.odometry.getPose().X()
        
        if pose_x > constants.FieldPos.op_wing_boundary:
            active_feed_zone = self.feed_zone_midline
        
        self.distance_to_feed_zone = (
            self.odometry.getPose().translation().distance(active_feed_zone) - constants.shooter_offset_y
        )
        
    
        # print("distance_to_target", self.distance_to_target)

        self.delta_z = (
                self.speaker_z - self.elevator.get_length() - constants.shooter_height 
        )
        
        if self.tuning:
            config.shot_height_offset = self.table.getNumber('shot height offset', config.shot_height_offset)
            
        self.delta_z += (config.shot_height_offset * inches_to_meters) + self.get_height_offset(self.distance_to_target)
        
        theta_1 = self.calculate_angle_no_air(self.distance_to_target, self.delta_z)
        
        feed_angle = self.calculate_angle_feed_zone(self.distance_to_feed_zone, self.get_shooter_height())
        if not self.use_air_resistance:
            self.shoot_angle = theta_1
            self.feed_angle = feed_angle
            self.t_total = self.distance_to_target / ((self.flywheel.get_velocity_linear() if self.flywheel.get_velocity_linear() > 0 else 1) * np.cos(theta_1))
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
    
    def get_rotation_to_feed_zone(self, pose:Pose2d=None, force_amp:bool=False):
        """
        returns rotation of base to face target
        :return: base target angle
        """
        
        feed_location = POI.Coordinates.Structures.Scoring.kFeed.getTranslation()
        
        robot_pose_2d = self.odometry.getPose() if pose == None else pose
        pose_x = robot_pose_2d.X()
        if not force_amp:
            if pose_x > constants.FieldPos.op_wing_boundary:
                feed_location = POI.Coordinates.Structures.Scoring.kFeedMidline.getTranslation()
        
        speaker_translation:Translation2d = feed_location
        
        robot_to_speaker = speaker_translation - robot_pose_2d.translation()
        return robot_to_speaker.angle()
    
    def get_shot_pos_tolerance(self):
        """
        returns tolerance of base to face target
        :return: base tolerance angle
        """
        speaker_translation:Translation2d = POI.Coordinates.Structures.Scoring.kSpeaker.getTranslation()
        
        bot_speaker_offset = config.speaker_length/2 - config.note_length/2
        
        bot_speaker_translation:Translation2d = speaker_translation - Translation2d(0, bot_speaker_offset)
        
        robot_pose_2d = self.odometry.getPose()
        bot_speaker_origin = bot_speaker_translation - robot_pose_2d.translation()
        small_angle = bot_speaker_origin.angle().radians()
        self.table.putNumber('bot speaker angle', degrees(small_angle))
        big_angle = self.get_rotation_to_speaker().radians()
        tolerance = abs(abs(small_angle) - abs(big_angle)) - radians(config.drivetrain_static_tolerance_offset)
        # return tolerance
        return min(
            max(tolerance, radians(config.min_drivetrain_tolerance)),
            radians(config.max_drivetrain_tolerance)
        )
    
    def get_shot_vel_tolerance(self):
        """
        returns tolerance of base to face target
        :return: base tolerance angle
        """
        tolerance = self.get_shot_pos_tolerance()
        pass

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
        self.feed_rotation2d = self.get_rotation_to_feed_zone()
        self.bot_shot_tolerance = self.get_shot_pos_tolerance()
        return self.base_rotation2d

    def update(self):
        """
        updates both shooter and base
        :return: base target angle
        """

        # self.update_shooter()
        # self.update_base()
        # self.update_tables()

    def update_tables(self):
        self.table.putNumber('wrist angle', degrees(self.get_theta()))
        self.table.putNumber('wrist feed angle', degrees(self.get_feed_theta()))
        self.table.putNumber('distance to target', self.distance_to_target)
        self.table.putNumber('bot angle', self.get_bot_theta().degrees())
        self.table.putNumber('bot feed angle', self.get_bot_theta_feed().degrees())
        self.table.putNumber('distance to feed zone', self.get_distance_to_feed_zone())
        self.table.putNumber('bot tolerance', degrees(self.get_shot_pos_tolerance()))
        self.table.putNumber('delta z', self.delta_z)
        self.table.putNumber('flywheel speed', self.get_flywheel_speed(self.distance_to_target))
        self.table.putNumber('feed flywheel speed', self.get_flywheel_speed_feed(self.distance_to_feed_zone))
        
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
        if self.tuning:
            config.shot_angle_offset = self.table.getNumber('shot angle offset', config.shot_angle_offset)
        
        self.shoot_angle = self.calculate_angle_no_air(self.get_distance_to_target(), self.get_delta_z())
        return self.shoot_angle + radians(config.shot_angle_offset)

    def get_feed_theta(self, force_amp: bool = False) -> radians:
        """
        Returns the angle of the trajectory.
        """
        self.feed_angle = self.calculate_angle_feed_zone(self.get_distance_to_feed_zone(force_amp=force_amp), self.get_shooter_height())
        return self.feed_angle
    
    def get_static_feed_theta(self) -> radians:
        """
        Returns the angle of the trajectory.
        """
        
        static_pose = POI.Coordinates.Structures.Scoring.kFeedStatic.get()
        
        self.feed_angle = self.calculate_angle_feed_zone(self.get_distance_to_feed_zone(static_pose, True), self.get_shooter_height())
        return self.feed_angle

    def get_bot_theta(self) -> Rotation2d:
        """
        Returns the angle of the Robot
        """
        self.base_rotation2d = self.get_rotation_to_speaker()
        return self.base_rotation2d
    
    def get_bot_theta_feed(self) -> Rotation2d:
        """
        Returns the angle of the Robot
        """
        self.feed_rotation2d = self.get_rotation_to_feed_zone()
        return self.feed_rotation2d
    
    def get_bot_theta_static_feed(self) -> Rotation2d:
        """
        Returns the angle of the Robot
        """
        
        static_pose = POI.Coordinates.Structures.Scoring.kFeedStatic.get()
        
        return self.get_rotation_to_feed_zone(static_pose, True)
    
    def get_distance_to_target(self) -> float:
        
        if type(self.speaker) == Translation3d:
            self.speaker = self.speaker.toTranslation2d()
        
        self.distance_to_target = (
            self.odometry.getPose().translation().distance(self.speaker) - constants.shooter_offset_y
        )
        return self.distance_to_target
    
    def get_distance_to_feed_zone(self, pose:Pose2d=None, force_amp:bool=False) -> float:
        
        if type(self.feed_zone) == Translation3d:
            self.feed_zone = self.feed_zone.toTranslation2d()
            
        if type(self.feed_zone_midline) == Translation3d:
            self.feed_zone_midline = self.feed_zone.toTranslation2d()

        
        active_feed_zone = self.feed_zone
        
        if not force_amp:
            pose_x = self.odometry.getPose().X()
            
            if pose_x > constants.FieldPos.op_wing_boundary:
                active_feed_zone = self.feed_zone_midline
        
        est_pose = self.odometry.getPose() if pose == None else pose
        
        self.distance_to_feed_zone = (
            est_pose.translation().distance(active_feed_zone) - constants.shooter_offset_y
        )
        
        return self.distance_to_feed_zone
    
    def get_distance_to_feed_zone_static(self, force_amp) -> float:
        
        static_pose = POI.Coordinates.Structures.Scoring.kFeedStatic.get()
        
        return self.get_distance_to_feed_zone(static_pose, force_amp)
    
    def get_delta_z(self) -> float:
        
        self.delta_z = (
                self.speaker_z - self.elevator.get_length() - constants.shooter_height 
        )
        
        if self.tuning:
            config.shot_height_offset = self.table.getNumber('shot height offset', config.shot_height_offset)
            
        self.delta_z += (config.shot_height_offset * inches_to_meters) + self.get_height_offset(self.distance_to_target)
        
        return self.delta_z

    def deriv(self, t, u):
        x, xdot, z, zdot = u
        speed = np.hypot(xdot, zdot)
        xdotdot = -self.k / constants.m * speed * xdot
        zdotdot = -self.k / constants.m * speed * zdot - constants.g
        return np.array([xdot, xdotdot, zdot, zdotdot])
