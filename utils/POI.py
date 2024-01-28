from __future__ import annotations
import constants
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d, Pose3d, Rotation3d, Transform3d
from units.SI import feet_to_meters, inches_to_meters, radians
import ntcore, math
from wpilib import DriverStation

    
class POIPose:
    
    _pose: Pose2d
    _red: bool
    # all poses are relative to the blue field origin
    def __init__(self, pose: Pose2d, red_origin: bool = False):
        self._pose = pose
        self._red = red_origin
        
    def __str__(self):
        return str(self._pose)
    
        
    def withRotation(self, radians: radians):
        return POIPose(Pose2d(self._pose.translation(), Rotation2d(radians)))
    
    def withOffset(self, offset: Translation2d):
        return POIPose(Pose2d(self._pose.translation() + offset, self._pose.rotation()))
    
    def __invertY(self, pose: Pose2d):
        invert = constants.field_width - pose.translation().Y()

        new_rotation = Rotation2d(-pose.rotation().radians())

        new_pose = Pose2d(Translation2d(pose.translation().X(), invert), new_rotation)
        return new_pose
    
    def get(self, verbose: bool = False):
        # if red is true, invert the y value
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed and not self._red\
            or DriverStation.getAlliance() == None and not self._red:
            self._red = True
            print("inverting") if verbose else None
            self._pose = self.__invertY(self._pose)
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue and self._red:
            self._red = False
            print("inverting") if verbose else None
            self._pose = self.__invertY(self._pose)
        return self._pose
    
    def getTranslation(self):
        return self.get(False).translation()

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
                kSpeaker = POIPose(Pose2d(
                    Translation2d(
                        0,
                        constants.FieldPos.Scoring.speaker_y
                    ), constants.FieldPos.pose_reverse))

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
                kStage = POIPose(Pose2d(
                    Translation2d(
                        constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.stage_length / 2,
                        constants.FieldPos.Stage.stage_y
                    ), constants.FieldPos.pose_reverse)
                    )


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
        
        