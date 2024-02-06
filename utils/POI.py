from __future__ import annotations
import constants, config
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d, Pose3d, Rotation3d, Transform3d, Translation3d, Twist2d
from units.SI import feet_to_meters, inches_to_meters, radians
import ntcore, math
from utils import LocalLogger
from wpilib import DriverStation
from typing import Callable, TypeVar, Literal

    
class POIPose:
    
    _pose: Pose2d | Pose3d
    _red: bool
    # all poses are relative to the blue field origin
    def __init__(self, pose: Pose2d | Pose3d, red_origin: bool = False):
        self._pose = pose
        if not isinstance(self._pose, Pose2d) and not isinstance(self._pose, Pose3d):
            raise TypeError("pose must be Pose2d or Pose3d")
        self._red = red_origin
        self.logger = LocalLogger("POI")
        
    def __str__(self):
        return str(self._pose)
    
        
    def withRotation(self, rotation: Rotation2d | Rotation3d | int | float):
        '''
        sets new rotation for the pose (float and int are interpreted as degrees)
        '''
        if isinstance(self._pose, Pose3d):
            if isinstance(rotation, float) or isinstance(rotation, int):
                rotation = Rotation3d(0,0,math.radians(rotation))
            return POIPose(Pose3d(self._pose.translation(), rotation), self._red)
        else:
            if isinstance(rotation, float) or isinstance(rotation, int):
                rotation = Rotation2d(math.radians(rotation))
            return POIPose(Pose2d(self._pose.translation(), rotation), self._red)
        
    def withRotationOffset(self, rotation: Rotation2d | Rotation3d | int | float):
        '''
        sets new rotation for the pose (float and int are interpreted as degrees)
        '''
        if isinstance(self._pose, Pose3d):
            if isinstance(rotation, float) or isinstance(rotation, int):
                rotation = Rotation3d(0,0,math.radians(rotation))
            return POIPose(Pose3d(self._pose.translation(), self._pose.rotation() + rotation), self._red)
        else:
            if isinstance(rotation, float) or isinstance(rotation, int):
                rotation = Rotation2d(math.radians(rotation))
            return POIPose(Pose2d(self._pose.translation(), self._pose.rotation() + rotation), self._red)
    
    def withOffset(self, offset: Translation2d | Translation3d):
        if isinstance(self._pose, Pose3d):
            return POIPose(Pose3d(self._pose.translation() + offset, self._pose.rotation()), self._red)
        else:
            return POIPose(Pose2d(self._pose.translation() + offset, self._pose.rotation()), self._red)
    
    def __invertY(self, pose: Pose2d | Pose3d):
        invert = constants.field_width - pose.translation().Y()
        new_rotation: Rotation2d | Rotation3d
        new_pose: Pose2d | Pose3d
        if isinstance(pose, Pose3d):
            new_rotation = Rotation3d(0, 0, -pose.rotation().Z())
            new_pose = Pose3d(Translation3d(pose.translation().X(), invert, pose.translation().Z()), new_rotation)
        else:
            new_rotation = Rotation2d(-pose.rotation().radians())
            new_pose = Pose2d(Translation2d(pose.translation().X(), invert), new_rotation)
        return new_pose
    
    def __check_inversion(self, verbose: bool = False):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed and not self._red\
            or DriverStation.getAlliance() == None and not self._red:
            self._red = True
            print("inverting") if verbose else None
            self._pose = self.__invertY(self._pose)
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue and self._red:
            self._red = False
            print("inverting") if verbose else None
            self._pose = self.__invertY(self._pose)
        else:
            print("not inverting") if verbose else None
    
    def get(self, verbose: bool = False):
        '''
        returns the pose2d, inverted if red is true
        '''
        # if red is true, invert the y value
        self.__check_inversion(verbose)
        if isinstance(self._pose, Pose3d):
            return self._pose.toPose2d()
        return self._pose
    
    def __str__(self):
        return str(self.get())
    
    def get3d(self, verbose: bool = True):
        '''
        returns the pose3d, inverted if red is true
        '''
        if not isinstance(self._pose, Pose3d):
            raise TypeError("pose must be Pose3d")
        # if red is true, invert the y value
        self.__check_inversion(verbose)
        return self._pose
    
    def getZ(self):
        if isinstance(self._pose, Pose3d):
            return self._pose.translation().Z()
        else:
            return 0
    
    def getTranslation(self):
        return self.get().translation()

class POI:
    _red: bool = False
    
    class Coordinates:
        
        class Notes:
            class Wing:
                
                kRight = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Wing.note_x,
                        constants.FieldPos.Wing.note_init
                    ), constants.FieldPos.pose_reverse))

                kCenter = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Wing.note_x,
                        constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap
                    ), constants.FieldPos.pose_reverse))

                kLeft = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Wing.note_x,
                        constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap * 2
                    ), constants.FieldPos.pose_reverse))


            class MidLine:
                kFarRight = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.MidLine.mid_line,
                        constants.FieldPos.MidLine.note_init
                    ), constants.FieldPos.pose_reverse))

                kMidRight = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.MidLine.mid_line,
                        constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap
                    ), constants.FieldPos.pose_reverse))

                kCenter = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.MidLine.mid_line,
                        constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 2
                    ), constants.FieldPos.pose_reverse))

                kMidLeft = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.MidLine.mid_line,
                        constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 3
                    ), constants.FieldPos.pose_reverse))

                kFarLeft = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.MidLine.mid_line,
                        constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 4
                    ), constants.FieldPos.pose_reverse))


        class Structures:

            class Scoring:
                
                kSpeaker = POIPose(Pose3d(
                    Translation3d(
                        0,
                        constants.FieldPos.Scoring.speaker_y,
                        constants.FieldPos.Scoring.speaker_z
                    ), Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians())
                ))

                kAmp = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Scoring.amp_x,
                        constants.FieldPos.Scoring.amp_y
                    ), constants.FieldPos.Scoring.amp_rotation))

                kAmpActual = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Scoring.amp_x,
                        constants.FieldPos.Scoring.amp_y_robot
                    ), constants.FieldPos.Scoring.amp_rotation))


            class Stage:
                kCenter = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Stage.stage_x,
                        constants.FieldPos.Stage.stage_y
                    ), constants.FieldPos.pose_reverse))

                kLeft = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.x_deviation,
                        constants.FieldPos.Stage.stage_y + constants.FieldPos.Stage.y_deviation
                    ), constants.FieldPos.Stage.left_rotation))

                kRight = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.x_deviation,
                        constants.FieldPos.Stage.stage_y - constants.FieldPos.Stage.y_deviation
                    ), constants.FieldPos.Stage.right_rotation))

            class Pickup:
                source = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Source.source_x,
                        constants.FieldPos.Source.source_y
                    ), constants.FieldPos.Source.rotation))

            class Obstacles:
                
                # Obstacles are usually in Pose3d, with the z acting as the recommended minimum distance to avoid the obstacle
                
                kStage = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.stage_length / 2 + constants.FieldPos.Stage.post_deviation,
                        constants.FieldPos.Stage.stage_y
                    ), constants.FieldPos.pose_reverse))
                
                kStageCenterPost = POIPose(Pose3d(
                    Translation3d(
                        constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.stage_length + constants.FieldPos.Stage.post_deviation,
                        constants.FieldPos.Stage.stage_y,
                        config.post_avoidance_distance,
                    ), Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians())))
                
                kStageLeftPost = POIPose(Pose3d(
                    Translation3d(
                        constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.post_deviation,
                        constants.FieldPos.Stage.stage_y + constants.FieldPos.Stage.stage_width / 2 - constants.FieldPos.Stage.post_deviation,
                        config.post_avoidance_distance,
                    ), Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians())))
                
                kStageRightPost = POIPose(Pose3d(
                    Translation3d(
                        constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.post_deviation,
                        constants.FieldPos.Stage.stage_y - constants.FieldPos.Stage.stage_width / 2 + constants.FieldPos.Stage.post_deviation,
                        config.post_avoidance_distance,
                    ), Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians())))


        class Waypoints:
            pass

    def __init__(self):
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Odometry")
        self.table = self.nt.getSubTable("POI")
        self.setNTValues()

    def setNTValues(self):
        # set NT values for all coordinates dynamically, as a list
        
        # get all classes in Coordinates
        classes = [cls for cls in self.Coordinates.__dict__.values() if isinstance(cls, type)]
        # find variables in all classes that are POIPose
        
        for cls in classes:
            
            table = self.table.getSubTable(cls.__name__)
            for classvar in cls.__dict__.values():
                list_of_POIPoses = []
                if isinstance(classvar, type):
                    for var in classvar.__dict__.values():
                        if isinstance(var, POIPose):
                            pose = var.get(False)
                            list_of_POIPoses += [pose.X(), pose.Y(), pose.rotation().radians()]
                            
                    table.putNumberArray(classvar.__name__, list_of_POIPoses)
        
        
        
def avoid_obstacles(start: POIPose, end: POIPose, obstacles: list[POIPose] | list[tuple[POIPose, float]], level: int = 0):
    '''
    returns a list of Transform2d that avoid obstacles. This should be used in the waypoints list
    '''
    table = ntcore.NetworkTableInstance.getDefault().getTable("Auto")
    if level == 0:
        table.putNumberArray('active obstacles', [])
        print('starting obstacle avoidance algorithm')
    
    if isinstance(obstacles[0], POIPose):
        obstacles:list[tuple[Pose2d, float]] = [(obstacles.get(False), obstacles.getZ()) for obstacles in obstacles]
    if level == 0:
        print('obstacles', obstacles)
    all_obstacles = obstacles.copy()
    # first, check if the start and end are within the general area of the obstacles
    # if not, remove from the list
    
    def between(a: float, b: float, c: float):
        return b - .25 <= a <= c + .25 or c - .25 <= a <= b + .25
    
    # print('checking if start and end are within the general area of the obstacles')
    
    def get_obstacles_in_range(obstacles: list[tuple[Pose2d, float]], start: POIPose, end: POIPose):
        potential_obstacles = []
        for obstacle in obstacles:
            if between(obstacle[0].X(), start.get(False).X(), end.get(False).X()) or between(obstacle[0].Y(), start.get().Y(), end.get().Y()):
                potential_obstacles.append(obstacle)
            else:    
                print("removed obstacle", obstacle)
                
        return potential_obstacles
    
    obstacles = get_obstacles_in_range(obstacles, start, end)
    
    if obstacles == []:
        print('no obstacles in range')
        return []
    
    # print('checked if start and end are within the general area of the obstacles')
    # next, check if the start and end are within the obstacles distance
    
    
    
    tally = 0
    for obstacle in obstacles:
        if obstacle[0].translation().distance(start.get(False).translation()) < obstacle[1] or obstacle[0].translation().distance(end.get(False).translation()) < obstacle[1]:
            # raise ValueError("start or end is within the obstacle distance")
            tally +=1
            # TODO: add code to move the start and/or end away from the obstacle
    
    if tally > 1:
        print('WARNING: start and end are within the obstacle distance')
    
    # next, check if the line between the start and end intersects with any of the obstacles
    
    def segment_intersects_circle(line: tuple[Translation2d, Translation2d], circle: tuple[Translation2d, float]):
        
        AC:Translation2d = circle[0] - line[0]
        AB:Translation2d = line[1] - line[0]
        # print(AB, AC, circle[1])
        def dot(v1: Translation2d, v2: Translation2d):
            return v1.X() * v2.X() + v1.Y() * v2.Y()
        
        # check if the circle is within the line segment
        if dot(AC, AB) <= 0:
            # print('circle is within the line segment')
            if circle[0].distance(line[0]) <= circle[1]:
                print('circle is within the line segment')
                return line[0]
            
        
        
        # get D by projecting AC onto AB and adding the start point
        d_proj = dot(AC, AB) / dot(AB, AB)
        D:Translation2d = line[0] + AB * d_proj
        # print(D)
        AD = D - line[0]
        
        # check if line segment is greater
        
        
        # print(AD)
        k = AD.X() / AB.X() if abs(AB.X()) > abs(AB.Y()) else AD.Y() / AB.Y()
        
        # get the point of the distance along AD
        
        dist:float = circle[0].distance(D)
        if k >= 1:
            dist = circle[0].distance(line[1])
        elif k <= 0:
            dist = circle[0].distance(line[0])
            
        # print(dist, circle[1])
            
        if dist <= circle[1]:
            print('segment intersects circle', dist, circle[1])
            # get the translation of the point of the distance of circle[0] that intersects the line
            
            # get the angle between AB and AC
            angle = math.atan2(AB.Y(), AB.X()) - math.atan2(AC.Y(), AC.X())
            if AB.X() == 0:
                return get_reciporical_translation(circle[0], (AB.X(), AB.Y()), angle, circle[1])
            else:
                return get_reciporical_translation(circle[0], (AB.X(), AB.Y()), angle, circle[1])
        return None
    
    def get_reciporical_translation(point: Translation2d, deltas: tuple[float, float], ans: float, magnitude: float):
        '''
        returns the translation of the point that is on the line of the deltas and slope
        '''

        angle = math.pi - math.atan2(deltas[0], deltas[1]) if start._red else -math.atan2(deltas[0], deltas[1])
        print(math.degrees(angle))
        print('ans', ans)
        ans = point + Translation2d(math.cos(angle) * magnitude, math.sin(angle) * magnitude)
        # print(point.distance(ans), magnitude)
        return ans
    
    def recursive_check(start: POIPose, end: POIPose, new_waypoint: Translation2d, remove_obs: list[tuple[Pose2d, float]]):
        point = POIPose(Pose2d(new_waypoint, Rotation2d(0)), start._red)
        new_obs = all_obstacles.copy()
        for obs in remove_obs:
            new_obs.remove(obs)
        print('new obstacle list', new_obs)
        if new_obs == []:
            return [new_waypoint]
        first_waypoints = avoid_obstacles(start, point, new_obs, level + 1)
        second_waypoints = avoid_obstacles(point, end, new_obs, level + 1)
        ans = first_waypoints + [new_waypoint] + second_waypoints
        print(ans, 'level recursive', level)
        return ans
    
    def get_obstacle_avoid_waypoints(start: POIPose, end: POIPose, obstacle: tuple[Pose2d, float]):
        '''
        returns a list of waypoints that avoid the obstacle
        '''
        waypoints = []
        res = segment_intersects_circle((start.getTranslation(), end.getTranslation()), (obstacle[0].translation(), obstacle[1]))
        if not res is None:
            print("intersection", 'new point', res)
            # waypoints += recursive_check(start, end, res, [obstacle])
            waypoints += [res]
            table.putNumberArray('active obstacles', table.getNumberArray('active obstacles', []) + [obstacle[0].X(), obstacle[0].Y(), 0])
            print(waypoints, 'level', level)
        return waypoints
    
    
        
    def get_waypoints():
        fin_waypoints:list[Translation2d] = []
        first_order_waypoints = []
        used_obstacles = []
        for obstacle in all_obstacles:
            res = get_obstacle_avoid_waypoints(start, end, obstacle)
            if res != []:
                first_order_waypoints += res
                used_obstacles.append(obstacle)
            # overlaping obstacles need to be removed before trying new ones
            
        for waypoint in first_order_waypoints:
            fin_waypoints += recursive_check(start, end, waypoint, used_obstacles)
        
        return fin_waypoints
        
    fin_waypoints = get_waypoints()
        
    if level == 0:
        print('final waypoints', fin_waypoints)
        nt_points = []
        for point in fin_waypoints:
            nt_points += [point.X(), point.Y(), 0]
        table.putNumberArray("waypoints", nt_points)
    return fin_waypoints
