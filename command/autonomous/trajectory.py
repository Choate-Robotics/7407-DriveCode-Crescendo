from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
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
        start_pose: Pose2d | POIPose | PoseType.current,
        waypoints: list[Translation2d] | list[POIPose],
        end_pose: Pose2d | POIPose,
        max_velocity: float,
        max_accel: float,
        start_velocity: float = 0,
        end_velocity: float = 0,
        rev: bool = False,
        obstacles: list[POIPose] = [Field.POI.Coordinates.Structures.Obstacles.kStageCenterPost, Field.POI.Coordinates.Structures.Obstacles.kStageRightPost, Field.POI.Coordinates.Structures.Obstacles.kStageLeftPost]
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
        
    def generate(self):
        
        # self.waypoints = avoid_obstacles(self.start_pose, self.end_pose, self.obstacles)
        
        active_start_pose, active_waypoints, active_end_pose = self.start_pose,[], self.end_pose
        
        if isinstance(self.start_pose, POIPose):
            active_start_pose = self.start_pose.get()
            
        for i, waypoint in enumerate(self.waypoints):
            if isinstance(waypoint, POIPose):
                active_waypoints = waypoint.get().translation()

        if isinstance(self.end_pose, POIPose):
            active_end_pose = self.end_pose.get()
        
        config = TrajectoryConfig(
            self.max_velocity,
            self.max_accel,
        )
        config.setStartVelocity(self.start_velocity)
        config.setEndVelocity(self.end_velocity)
        config.setReversed(self.rev)
        
        
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            start=active_start_pose,
            interiorWaypoints=active_waypoints,
            end=active_end_pose,
            config=config,
        )
        return self.trajectory