from __future__ import annotations
import constants
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d, Pose3d, Rotation3d, Transform3d
from units.SI import feet_to_meters, inches_to_meters
import ntcore, math


class POI:
    
    
    
    _red: bool = False
    
    # all poses are relative to the blue field origin
    class Notes:
        
        class Wing:
            
            kLeft:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Wing.note_x,
                    constants.FieldPos.Wing.note_init
                    ), constants.FieldPos.pose_reverse)
            
            kRight:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Wing.note_x,
                    constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap
                    ), constants.FieldPos.pose_reverse)
            
            kCenter:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Wing.note_x,
                    constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap * 2
                    ), constants.FieldPos.pose_reverse)
            
        class MidLine:
            
            kFar_left:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line, 
                    constants.FieldPos.MidLine.note_init
                    ), constants.FieldPos.pose_reverse)
            
            kMid_left:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap
                    ), constants.FieldPos.pose_reverse)
            
            kCenter:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 2
                    ), constants.FieldPos.pose_reverse)
            
            kMid_right:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 3
                    ), constants.FieldPos.pose_reverse)
            
            kFar_right:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 4
                    ), constants.FieldPos.pose_reverse)
    
    class Structures:
        
        class Scoring:
            
            kSpeaker:Pose2d = Pose2d(
                Translation2d(
                    0,
                    constants.FieldPos.Scoring.speaker_y
                    ), constants.FieldPos.pose_reverse)
            
            kAmp:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Scoring.amp_x,
                    constants.FieldPos.Scoring.amp_y
                    ), constants.FieldPos.Scoring.amp_rotation)
            
            kAmpActual:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Scoring.amp_x,
                    constants.FieldPos.Scoring.amp_y_robot
                    ), constants.FieldPos.Scoring.amp_rotation)
            
        class Stage:
            
            center:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Stage.stage_x,
                    constants.FieldPos.Stage.stage_y
                    ), constants.FieldPos.pose_reverse)
            
            left:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.x_deviation,
                    constants.FieldPos.Stage.stage_y + constants.FieldPos.Stage.y_deviation
                    ), constants.FieldPos.Stage.left_rotation)
            
            right:Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.x_deviation,
                    constants.FieldPos.Stage.stage_y - constants.FieldPos.Stage.y_deviation
                    ), constants.FieldPos.Stage.right_rotation)
            
        class Pickup:
            
            source:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
        class Obstacles:
            pass
            
    
    def __init__(self):
        self._red = False
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt.getTable("Odometry")
        
    def getPoses(self, Object: object):
        poses: list[Pose2d] = list(
                {k: v for k, v in Object.__dict__.items() if isinstance(v, Pose2d)}.values()
            )
        
        return poses
        
    def setValues(self):
        
        POI = []
        
        def append(pose:Pose2d):
            x = pose.translation().X()
            y = pose.translation().Y()
            theta = pose.rotation().radians()
            return [x, y, theta]
        
        for note in self.getPoses(self.Notes.Wing):
            POI += append(note)
            
        for note in self.getPoses(self.Notes.MidLine):
            POI += append(note)
            
        for structure in self.getPoses(self.Structures.Scoring):
            POI += append(structure)
            
        for stage in self.getPoses(self.Structures.Stage):
            POI += append(stage)
            
        for pickup in self.getPoses(self.Structures.Pickup):
            POI += append(pickup)
            
        self.table.putNumberArray("POI", POI)
        
        
    def init(self):
        
        self.setValues()
        
    def invertY(self, pose:Pose2d):
        invert = self.field_width - pose.translation().Y()
        print(invert)
        new_pose = Pose2d(Translation2d(pose.translation().X(), invert), pose.rotation())
        return new_pose
    
    def invert(self):
        for note in self.getPoses(self.Notes.Wing):
            note = self.invertY(note)
        
        for note in self.getPoses(self.Notes.MidLine):
            note = self.invertY(note)
            
        for structure in self.getPoses(self.Structures.Scoring):
            structure = self.invertY(structure)
            
        for stage in self.getPoses(self.Structures.Stage):
            stage = self.invertY(stage)
            
        for pickup in self.getPoses(self.Structures.Pickup):
            pickup = self.invertY(pickup)
            
        self.setValues()
        print(self.Notes.Wing.left.translation().Y())
        
    def setRed(self):
        print("setting red")
        if self._red:
            return
        self.invert()
        self._red = True
        
    def setBlue(self):
        print("setting blue")
        if not self._red:
            return
        self.invert()
        self._red = False
        