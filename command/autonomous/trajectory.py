from enum import Enum

from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

# import config
from robot_systems import Field
from utils import POIPose
import config
from enum import Enum

class PoseType(Enum):
    
    current = 0


class CustomTrajectory:
    """
    Wrapper for the wpimath trajectory class with additional info.

    :param start_pose: Starting pose.
    :type start_pose: Pose2d
    :param waypoints: Waypoints.
    :type waypoints: list[Translation2d]
    :param end_pose: Ending pose.
    :type end_pose: Pose2d
    :param max_velocity: Maximum velocity.
    :type max_velocity: float (meters per second)
    :param max_accel: Maximum acceleration.
    :type max_accel: float (meters per second squared)
    :param start_velocity: Starting velocity.
    :type start_velocity: float (meters per second)
    :param end_velocity: Ending velocity.
    :type end_velocity: float (meters per second)
    """

    def __init__(
        self,
        start_pose: Pose2d | POIPose | PoseType,
        waypoints: list[Translation2d] | list[POIPose],
        end_pose: Pose2d | POIPose,
        max_velocity: float,
        max_accel: float,
        start_velocity: float = 0,
        end_velocity: float = 0,
        rev: bool = False,
        obstacles: list[POIPose] = [
            Field.POI.Coordinates.Structures.Obstacles.kStageCenterPost,
            Field.POI.Coordinates.Structures.Obstacles.kStageRightPost,
            Field.POI.Coordinates.Structures.Obstacles.kStageLeftPost,
        ],
        start_rotation: float = None
    ):
        self.start_pose = start_pose
        self.waypoints = waypoints
        self.end_pose = end_pose
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.start_velocity = start_velocity
        self.end_velocity = end_velocity
        self.rev = rev
        self.obstacles = obstacles
        self.start_rotation = start_rotation

    def generate(self):
        # self.waypoints = avoid_obstacles(self.start_pose, self.end_pose, self.obstacles)
        
        waypoints: list[Translation2d] = []
        
        active_start_pose, active_waypoints, active_end_pose = self.start_pose, waypoints, self.end_pose

        if self.start_rotation != None:
            active_start_rotation = self.start_rotation
            if config.active_team == config.Team.BLUE:
                active_start_rotation *= -1
        
        if isinstance(self.start_pose, POIPose):
            active_start_pose = self.start_pose.get()

        if isinstance(self.start_pose, PoseType):
            if self.start_pose == PoseType.current:
                active_start_pose = Pose2d(Field.odometry.getPose().translation(), Rotation2d(active_start_rotation))
            else:
                raise ValueError('Invalid PoseType')
            
        for i, waypoint in enumerate(self.waypoints):
            if isinstance(waypoint, POIPose):
                active_waypoints += [waypoint.get().translation()]
            elif isinstance(waypoint, Translation2d):
                active_waypoints += [waypoint]

        if isinstance(self.end_pose, POIPose):
            active_end_pose = self.end_pose.get()
        
        config_ = TrajectoryConfig(
            self.max_velocity,
            self.max_accel,
        )
        config_.setStartVelocity(self.start_velocity)
        config_.setEndVelocity(self.end_velocity)
        config_.setReversed(self.rev)

        self.trajectory = TrajectoryGenerator.generateTrajectory(
            start=active_start_pose,
            interiorWaypoints=active_waypoints,
            end=active_end_pose,
            config=config_,
        )
        return self.trajectory
    
    
# class CustomTrajectoryAutoIntake:
    
#     def __init__(
#         self,
#         start_pose: Pose2d | POIPose,
#         waypoints: list[Translation2d] | list[POIPose],
#         note_detect_pose: Pose2d | POIPose,
#         alt_pose: Pose2d | POIPose,
#         max_velocity: float,
#         max_accel: float,
#         start_velocity: float = 0,
#         end_velocity: float = 0,
#         rev: bool = False,
#     ):
#         self.start_pose = start_pose
#         self.waypoints = waypoints
#         self.note_detect_pose = note_detect_pose
#         self.alt_pose = alt_pose
#         self.max_velocity = max_velocity
#         self.max_accel = max_accel
#         self.start_velocity = start_velocity
#         self.end_velocity = end_velocity
#         self.rev = rev
        
#         self.base_exit_vel = min(self.end_velocity, config.object_detection_drivetrain_speed_dx * constants.drivetrain_max_vel)

#         self.base_trajectory = CustomTrajectory(
#             self.start_pose,
#             self.waypoints,
#             self.note_detect_pose,
#             self.max_velocity,
#             self.max_accel,
#             self.start_velocity,
#             self.base_exit_vel,
#             self.rev
#         )
        
#         self.alt_trajectory = CustomTrajectory(
#             self.note_detect_pose, # eventually replace with current pose
#             [],
#             self.alt_pose, 
#             self.max_velocity,
#             self.max_accel,
#             self.base_exit_vel,
#             self.end_velocity,
#             self.rev
#         )
        
#     def get_base(self):
#         return self.base_trajectory
    
#     def get_alt(self):
#         return self.alt_trajectory


