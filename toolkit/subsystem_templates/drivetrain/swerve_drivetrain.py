import math

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)

from toolkit.oi.joysticks import JoystickAxis
from toolkit.sensors.gyro import BaseGyro
from toolkit.subsystem import Subsystem
from toolkit.utils import logger
from toolkit.utils.toolkit_math import bounded_angle_diff
from units.SI import (
    degrees,
    degrees_per_second__to__radians_per_second,
    meters,
    meters_per_second,
    miles_per_hour_to_meters_per_second,
    radians,
    radians_per_second,
    rotations_per_second__to__radians_per_second,
)


class SwerveNode:
    """
    Extendable class for swerve node.
    """

    motor_reversed: bool = False
    motor_sensor_offset: radians = 0

    def init(self):
        """
        Initialize the swerve node.
        """
        ...

    def set(self, vel: meters_per_second, angle_radians: radians_per_second):
        """
        Set the velocity and angle of the swerve node.

        Args:
            vel (meters_per_second): velocity of the swerve node
            angle_radians (radians_per_second): turning swerve node velocity in radians per second
        """
        self._set_angle(
            angle_radians, self.get_turn_motor_angle() + self.motor_sensor_offset
        )
        self.set_motor_velocity(vel if not self.motor_reversed else -vel)

    # OVERRIDDEN FUNCTIONS
    def set_motor_angle(self, pos: radians):
        """
        Set the angle of the swerve node. Must be overridden.

        Args:
            pos (radians): angle of the swerve node in radians
        """
        ...

    def get_turn_motor_angle(self) -> radians:
        """
        Get the current angle of the swerve node. Must be overridden. Must return radians.
        """
        ...

    def get_abs(self):
        """
        Gets the absolute encoder value. Must be overridden.
        """
        ...

    def set_motor_velocity(self, vel: meters_per_second):
        """
        Set the velocity of the swerve node. Must be overridden.
        Args:
            vel (meters_per_second): velocity of the swerve node in meters per second
        """
        ...

    def get_motor_velocity(self) -> meters_per_second:
        """
        Get the velocity of the swerve node. Must be overridden. Must return meters per second.
        """
        ...

    def get_drive_motor_traveled_distance(self) -> meters:
        """
        Get the distance traveled by the drive motor. Must be overridden. Must return meters.
        """
        ...

    def get_node_position(self) -> SwerveModulePosition:
        """
        Get the position of the swerve node.

        Returns:
            SwerveModulePosition: position of the swerve node
        """
        return SwerveModulePosition(
            self.get_drive_motor_traveled_distance(),
            Rotation2d(self.get_turn_motor_angle()),
        )

    def get_node_state(self) -> SwerveModuleState:
        """
        Get the state of the swerve node.
        Returns:
            SwerveModuleState: state of the swerve node
        """
        return SwerveModuleState(
            self.get_motor_velocity(), Rotation2d(self.get_turn_motor_angle())
        )

    # 0 degrees is facing right | "ethan is our FRC lord and saviour" - sid
    def _set_angle(self, target_angle: radians, initial_angle: radians):
        target_sensor_angle, flipped, flip_sensor_offset = SwerveNode._resolve_angles(
            target_angle, initial_angle
        )

        target_sensor_angle -= self.motor_sensor_offset

        if flipped:
            self.motor_reversed = not self.motor_reversed
            self.motor_sensor_offset += flip_sensor_offset

        self.set_motor_angle(target_sensor_angle)

    @staticmethod
    def _resolve_angles(
        target_angle: radians, initial_angle: radians
    ) -> tuple[float, bool, float]:
        """
        :param target_angle: Target node angle
        :param initial_angle: Initial node sensor angle
        :return: (target_sensor_angle, flipped, flip_sensor_offset)
        """

        # Actual angle difference in radians
        diff = bounded_angle_diff(initial_angle, target_angle)

        # Should we flip
        if abs(diff) > 0.65 * math.pi:
            flip_sensor_offset = math.pi if diff > 0 else -math.pi
            diff -= flip_sensor_offset
            return diff + initial_angle, True, flip_sensor_offset

        return diff + initial_angle, False, 0


class SwerveGyro(BaseGyro):
    """
    Extendable class for swerve gyro.
    """

    def __init__(self):
        super().__init__()


class SwerveDrivetrain(Subsystem):
    """
    Swerve Drivetrain Extendable class. Contains driving functions.
    """

    n_front_left: SwerveNode
    n_front_right: SwerveNode
    n_back_left: SwerveNode
    n_back_right: SwerveNode
    gyro: SwerveGyro
    axis_dx: JoystickAxis
    axis_dy: JoystickAxis
    axis_rotation: JoystickAxis
    track_width: meters = 1
    max_vel: meters_per_second = 20 * miles_per_hour_to_meters_per_second
    max_angular_vel: radians_per_second = (
        4 * rotations_per_second__to__radians_per_second
    )
    deadzone_velocity: meters_per_second = 0.05  # Does not run within this speed
    deadzone_angular_velocity: radians_per_second = (
        5 * degrees_per_second__to__radians_per_second
    )  # Will not turn within this speed
    start_pose: Pose2d = Pose2d(
        0, 0, 0
    )  # Starting pose of the robot from wpilib Pose (x, y, rotation)
    gyro_start_angle: radians = 0
    gyro_offset: degrees = 0

    def __init__(self):
        super().__init__()
        self.kinematics: SwerveDrive4Kinematics | None = None
        self.odometry: SwerveDrive4Odometry | None = None
        self.odometry_estimator: SwerveDrive4PoseEstimator | None = None
        self.chassis_speeds: ChassisSpeeds | None = ChassisSpeeds(0, 0, 0)
        self._omega: radians_per_second = 0

        self.node_translations: tuple[Translation2d] | None = None

    def init(self):
        """
        Initialize the swerve drivetrain, kinematics, odometry, and gyro.
        """
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_front_left.init()
        self.n_front_right.init()
        self.n_back_left.init()
        self.n_back_right.init()
        self.gyro.init(self.gyro_start_angle)

        logger.info("initializing odometry", "[swerve_drivetrain]")

        self.node_translations = (
            Translation2d(0.5 * self.track_width, 0.5 * self.track_width),
            Translation2d(0.5 * self.track_width, -0.5 * self.track_width),
            Translation2d(-0.5 * self.track_width, 0.5 * self.track_width),
            Translation2d(-0.5 * self.track_width, -0.5 * self.track_width),
        )

        self.kinematics = SwerveDrive4Kinematics(*self.node_translations)

        self.odometry = SwerveDrive4Odometry(
            self.kinematics, self.get_heading(), self.node_positions, self.start_pose
        )
        self.odometry_estimator = SwerveDrive4PoseEstimator(
            self.kinematics, self.get_heading(), self.node_positions, self.start_pose
        )

        logger.info("initialization complete", "[swerve_drivetrain]")

    @property
    def node_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        """
        Get the node positions.
        """
        return (
            self.n_front_left.get_node_position(),
            self.n_front_right.get_node_position(),
            self.n_back_left.get_node_position(),
            self.n_back_right.get_node_position(),
        )

    @property
    def node_states(
        self,
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        """
        Get the node states.
        """
        return (
            self.n_front_left.get_node_state(),
            self.n_front_right.get_node_state(),
            self.n_back_left.get_node_state(),
            self.n_back_right.get_node_state(),
        )

    def set_driver_centric(
        self,
        vel: (meters_per_second, meters_per_second),
        angular_vel: radians_per_second,
    ):
        """
        Set the driver centric velocity and angular velocity. Driver centric runs with perspective of driver.

        Args:
            vel: velocity in x and y direction as (meters per second, meters per second)
            angular_vel: angular velocity in radians per second
        """
        # vel = rotate_vector(vel[0], vel[1], -self.gyro.get_robot_heading())

        robo_speed = ChassisSpeeds.fromFieldRelativeSpeeds(
            vel[0], vel[1], angular_vel, self.get_heading()
        )

        vel = robo_speed.vx, robo_speed.vy

        self.set_robot_centric(vel, angular_vel)

    def set_robot_centric(
        self,
        vel: (meters_per_second, meters_per_second),
        angular_vel: radians_per_second,
    ):
        """
        Set the robot centric velocity and angular velocity. Robot centric runs with perspective of robot.
        Args:
            vel: velocity in x and y direction as (meters per second, meters per second)
            angular_vel: angular velocity in radians per second
        """

        dx, dy = vel

        # if abs(dx) < self.deadzone_velocity and abs(dy) < self.deadzone_velocity and \
        #         abs(angular_vel) < self.deadzone_angular_velocity:
        #             pass
        #     # self.n_front_left.set_motor_velocity(0)
        #     # self.n_front_right.set_motor_velocity(0)
        #     # self.n_back_left.set_motor_velocity(0)
        #     # self.n_back_right.set_motor_velocity(0)
        # else:

        dx = 0 if abs(dx) < self.deadzone_velocity else dx

        dy = 0 if abs(dy) < self.deadzone_velocity else dy

        angular_vel = (
            0 if abs(angular_vel) < self.deadzone_angular_velocity else angular_vel
        )

        self.chassis_speeds = ChassisSpeeds(dx, dy, -angular_vel)

        new_states = self.kinematics.toSwerveModuleStates(self.chassis_speeds)
        normalized_states = self.kinematics.desaturateWheelSpeeds(
            new_states, self.max_vel
        )

        # normalized_states = new_states
        fl, fr, bl, br = normalized_states

        self.n_front_left.set(fl.speed, fl.angle.radians())
        self.n_front_right.set(fr.speed, fr.angle.radians())
        self.n_back_left.set(bl.speed, bl.angle.radians())
        self.n_back_right.set(br.speed, br.angle.radians())

        self._omega = angular_vel  # For simulation

        # self.odometry.update(
        #     self.get_heading(),
        #     self.node_positions
        # )

        # self.odometry_estimator.update(
        #     self.get_heading(),
        #     self.node_positions
        # )

        # self.chassis_speeds = self.kinematics.toChassisSpeeds(*self.node_states)

    def stop(self):
        """
        Stop the drivetrain and all pods.
        """
        self.n_front_left.set(0, 0)
        self.n_front_right.set(0, 0)
        self.n_back_left.set(0, 0)
        self.n_back_right.set(0, 0)

    def get_heading(self) -> Rotation2d:
        """
        Get the robot heading.

        Returns:
            Heading (Rotation2d): the robot heading
        """
        return Rotation2d(self.gyro.get_robot_heading() + self.gyro_offset)

    def reset_odometry(self, pose: Pose2d):
        """
        Reset the odometry to a given pose.

        Args:
            pose (Pose2d): The pose to reset the odometry to.
        """
        self.odometry.resetPosition(self.get_heading(), pose, *self.node_positions)
        self.odometry_estimator.resetPosition(
            gyroAngle=self.get_heading(), pose=pose, modulePositions=self.node_positions
        )

    @staticmethod
    def _calculate_swerve_node(
        node_x: meters,
        node_y: meters,
        dx: meters_per_second,
        dy: meters_per_second,
        d_theta: radians_per_second,
    ) -> (meters_per_second, radians):
        tangent_x, tangent_y = -node_y, node_x
        tangent_m = math.sqrt(tangent_x**2 + tangent_y**2)
        tangent_x /= tangent_m
        tangent_y /= tangent_m

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * tangent_x
        sy = dy + r * d_theta * tangent_y

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx**2 + sy**2)
        return magnitude, theta


# class SwerveDrivetrain(Subsystem):
#     """
#     Swerve Drivetrain Extendable class. Contains driving functions.
#     """
#     n_front_left: SwerveNode
#     n_front_right: SwerveNode
#     n_back_left: SwerveNode
#     n_back_right: SwerveNode
#     gyro: SwerveGyro
#     axis_dx: JoystickAxis
#     axis_dy: JoystickAxis
#     axis_rotation: JoystickAxis
#     track_width: meters = 1
#     max_vel: meters_per_second = 20 * miles_per_hour_to_meters_per_second
#     max_angular_vel: radians_per_second = 4 * rotations_per_second__to__radians_per_second
#     deadzone_velocity: meters_per_second = 0.05  # Does not run within this speed
#     deadzone_angular_velocity: radians_per_second = 5 * degrees_per_second__to__radians_per_second
#     Will not turn within this speed
#     start_pose: Pose2d = Pose2d(0, 0, 0)  # Starting pose of the robot from wpilib Pose (x, y, rotation)
#     gyro_start_angle: radians = 0
#     gyro_offset: degrees = 0

#     def __init__(self):
#         super().__init__()
#         self.kinematics: SwerveDrive4Kinematics | None = None
#         self.odometry: SwerveDrive4Odometry | None = None
#         self.odometry_estimator: SwerveDrive4PoseEstimator | None = None
#         self.chassis_speeds: ChassisSpeeds | None = None
#         self._omega: radians_per_second = 0

#         self.node_translations: tuple[Translation2d] | None = None

#     def init(self):
#         """
#         Initialize the swerve drivetrain, kinematics, odometry, and gyro.
#         """
#         logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
#         self.n_front_left.init()
#         self.n_front_right.init()
#         self.n_back_left.init()
#         self.n_back_right.init()
#         self.gyro.init(self.gyro_start_angle)

#         logger.info("initializing odometry", "[swerve_drivetrain]")

#         self.node_translations = (
#             Translation2d(.5 * self.track_width, .5 * self.track_width),
#             Translation2d(.5 * self.track_width, -.5 * self.track_width),
#             Translation2d(-.5 * self.track_width, .5 * self.track_width),
#             Translation2d(-.5 * self.track_width, -.5 * self.track_width)
#         )

#         self.kinematics = SwerveDrive4Kinematics(
#             *self.node_translations
#         )

#         self.odometry = SwerveDrive4Odometry(
#             self.kinematics,
#             self.get_heading(),
#             self.node_positions,
#             self.start_pose
#         )
#         self.odometry_estimator = SwerveDrive4PoseEstimator(
#             self.kinematics,
#             self.get_heading(),
#             self.node_positions,
#             self.start_pose
#         )


#         logger.info("initialization complete", "[swerve_drivetrain]")

#     @property
#     def node_positions(self) -> tuple[
#         SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition
#     ]:
#         """
#         Get the node positions.
#         """
#         return (
#             self.n_front_left.get_node_position(),
#             self.n_front_right.get_node_position(),
#             self.n_back_left.get_node_position(),
#             self.n_back_right.get_node_position()
#         )

#     @property
#     def node_states(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
#         """
#         Get the node states.
#         """
#         return (
#             self.n_front_left.get_node_state(),
#             self.n_front_right.get_node_state(),
#             self.n_back_left.get_node_state(),
#             self.n_back_right.get_node_state()
#         )

#     def set_driver_centric(self, vel: (meters_per_second, meters_per_second), angular_vel: radians_per_second):
#         """
#         Set the driver centric velocity and angular velocity. Driver centric runs with perspective of driver.

#         Args:
#             vel: velocity in x and y direction as (meters per second, meters per second)
#             angular_vel: angular velocity in radians per second
#         """
#         vel = rotate_vector(vel[0], vel[1], -self.gyro.get_robot_heading())
#         self.set_robot_centric(vel, angular_vel)

#     def set_robot_centric(self, vel: (meters_per_second, meters_per_second), angular_vel: radians_per_second):
#         """
#         Set the robot centric velocity and angular velocity. Robot centric runs with perspective of robot.
#         Args:
#             vel: velocity in x and y direction as (meters per second, meters per second)
#             angular_vel: angular velocity in radians per second
#         """
#         self._omega = angular_vel  # For simulation

#         if abs(vel[0]) < self.deadzone_velocity and abs(vel[1]) < self.deadzone_velocity and \
#                 abs(angular_vel) < self.deadzone_angular_velocity:
#             self.n_front_left.set_motor_velocity(0)
#             self.n_front_right.set_motor_velocity(0)
#             self.n_back_left.set_motor_velocity(0)
#             self.n_back_right.set_motor_velocity(0)
#         else:
#             self.n_front_left.set(*self._calculate_swerve_node(
#                 -.5 * self.track_width, -.5 * self.track_width,
#                 vel[0], vel[1], angular_vel
#             ))
#             self.n_front_right.set(*self._calculate_swerve_node(
#                 -.5 * self.track_width, .5 * self.track_width,
#                 vel[0], vel[1], angular_vel
#             ))
#             self.n_back_left.set(*self._calculate_swerve_node(
#                 .5 * self.track_width, -.5 * self.track_width,
#                 vel[0], vel[1], angular_vel
#             ))
#             self.n_back_right.set(*self._calculate_swerve_node(
#                 .5 * self.track_width, .5 * self.track_width,
#                 vel[0], vel[1], angular_vel
#             ))

#         self.odometry.update(
#             self.get_heading(),
#             *self.node_positions
#         )

#         self.odometry_estimator.update(
#             self.get_heading(),
#             self.node_positions
#         )

#         self.chassis_speeds = self.kinematics.toChassisSpeeds(*self.node_states)

#     def stop(self):
#         """
#         Stop the drivetrain and all pods.
#         """
#         self.n_front_left.set(0, 0)
#         self.n_front_right.set(0, 0)
#         self.n_back_left.set(0, 0)
#         self.n_back_right.set(0, 0)

#     def get_heading(self) -> Rotation2d:
#         """
#         Get the robot heading.

#         Returns:
#             Heading (Rotation2d): the robot heading
#         """
#         return Rotation2d(self.gyro.get_robot_heading() + self.gyro_offset)

#     def reset_odometry(self, pose: Pose2d):
#         """
#         Reset the odometry to a given pose.

#         Args:
#             pose (Pose2d): The pose to reset the odometry to.
#         """
#         self.odometry.resetPosition(
#             self.get_heading(),
#             pose,
#             *self.node_positions
#         )
#         self.odometry_estimator.resetPosition(
#             gyroAngle=self.get_heading(),
#             pose=pose,
#             modulePositions=self.node_positions
#         )

#     @staticmethod
#     def _calculate_swerve_node(node_x: meters, node_y: meters, dx: meters_per_second, dy: meters_per_second,
#                                d_theta: radians_per_second) -> (meters_per_second, radians):
#         tangent_x, tangent_y = -node_y, node_x
#         tangent_m = math.sqrt(tangent_x ** 2 + tangent_y ** 2)
#         tangent_x /= tangent_m
#         tangent_y /= tangent_m

#         r = math.sqrt(2) / 2
#         sx = dx + r * d_theta * tangent_x
#         sy = dy + r * d_theta * tangent_y

#         theta = math.atan2(sy, sx)
#         magnitude = math.sqrt(sx ** 2 + sy ** 2)
#         return magnitude, theta
