from __future__ import annotations
import constants
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d, Pose3d, Rotation3d, Transform3d
from units.SI import feet_to_meters
import ntcore


class POI:
    
    field_width = 8.21
    
    class Notes:
        
        class Wing:
            
            left:Pose2d = Pose2d(Translation2d(0.4, 6.4), Rotation2d(0))
            
            right:Pose2d = Pose2d(Translation2d(6.0, 3), Rotation2d(0))
            
            center:Pose2d = Pose2d(Translation2d(23, 0.4), Rotation2d(0))
            
        class MidLine:
            
            far_left:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
            mid_left:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
            center:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
            mid_right:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
            far_right:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
    
    class Structures:
        
        class Scoring:
            
            speaker:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
            amp:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
        class Stage:
            
            center:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
            left:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
            right:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
        class Pickup:
            
            source:Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))
            
    
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
        pose.translation().y = self.field_width - pose.translation().Y()
        return pose
        
    def setRed(self):
        if self._red:
            return
        
        for note in self.Notes.Wing:
            note = self.invertY(note)
        
        for note in self.Notes.MidLine:
            note = self.invertY(note)
            
        for structure in self.Structures.Scoring:
            structure = self.invertY(structure)
            
        for stage in self.Structures.Stage:
            stage = self.invertY(stage)
            
        for pickup in self.Structures.Pickup:
            pickup = self.invertY(pickup)
        
        self._red = True
        
        self.setValues()
        
    def setBlue(self):
        if not self._red:
            return
        self.setRed()
        self._red = False
        