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
        
        
        
def avoid_obstacles(start: POIPose, end: POIPose, obstacles: list[POIPose]):
    '''
    returns a list of Transform2d that avoid obstacles. This should be used in the waypoints list
    '''
    
    obstacles:list[tuple[Pose2d, float]] = [(obstacles.get(False), obstacles.getZ()) for obstacles in obstacles]
    print(obstacles)
    # first, check if the start and end are within the general area of the obstacles
    # if not, remove from the list
    
    def between(a: float, b: float, c: float):
        return b - 1 <= a <= c + 1 or c - 1 <= a <= b + 1
    
    print('checking if start and end are within the general area of the obstacles')
    
    for obstacle in obstacles:
        if not between(obstacle[0].X(), start.get(False).X(), end.get(False).X()) or not between(obstacle[0].Y(), start.get().Y(), end.get().Y()):
            obstacles.remove(obstacle)
            print("removed obstacle", obstacle)
    
    
    print('checked if start and end are within the general area of the obstacles')
    # next, check if the start and end are within the obstacles distance
    for obstacle in obstacles:
        if obstacle[0].translation().distance(start.get(False).translation()) < obstacle[1] or obstacle[0].translation().distance(end.get(False).translation()) < obstacle[1]:
            # raise ValueError("start or end is within the obstacle distance")
            print("start or end is within the obstacle distance")
            # TODO: add code to move the start and/or end away from the obstacle
        else:
            print("start and end are not within the obstacle distance")
    # next, check if the line between the start and end intersects with any of the obstacles
    
    def line_intersection(line1: tuple[Translation2d, Translation2d], line2: tuple[Translation2d, Translation2d]):
        '''
        returns true if the lines intersect
        '''
        def ccw(A: Translation2d, B: Translation2d, C: Translation2d):
            return (C.Y() - A.Y()) * (B.X() - A.X()) > (B.Y() - A.Y()) * (C.X() - A.X())
        
        A, B = line1
        C, D = line2
        return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
    
    def get_deltas(line: tuple[Translation2d, Translation2d], point: Translation2d):
        # check if point is above or below the line using standard form
        
        # get the standard form of the line
        A, B = line
        a = A.Y() - B.Y()
        b = B.X() - A.X()
        c = A.X() * B.Y() - B.X() * A.Y()
        
        # plug in the point to the standard form
        ans = a * point.X() + b * point.Y() + c
        
        # if ans is positive, the point is above the line
        # if ans is negative, the point is below the line
        
        if ans > 0:
            return (A.X() - B.X(), A.Y() - B.Y())
        else:
            return (B.X() - A.X(), B.Y() - A.Y())
        
    def get_reciporical_translation(point: Translation2d, deltas: tuple[float, float], magnitude: float):
        '''
        returns the translation of the point that is on the line of the deltas and slope
        '''
        angle = math.atan2(deltas[0], deltas[1])
        ans = point + Translation2d(math.cos(angle) * magnitude, math.sin(angle) * magnitude)
        print(point.distance(ans), magnitude)
        return ans
    
    waypoints = []
    print('checking if the line between the start and end intersects with any of the obstacles')
    for obstacle in obstacles:
        deltas = get_deltas((start.getTranslation(), end.getTranslation()), obstacle[0].translation())
        tangent_point = get_reciporical_translation(obstacle[0].translation(), deltas, obstacle[1])
        if line_intersection((start.getTranslation(), end.getTranslation()), (obstacle[0], tangent_point)):
            print("intersection", 'new point', tangent_point)
            waypoints.append(tangent_point)
            
    return waypoints
