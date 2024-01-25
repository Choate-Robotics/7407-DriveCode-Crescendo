from __future__ import annotations
import constants
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d, Pose3d, Rotation3d, Transform3d
from units.SI import feet_to_meters, inches_to_meters
import ntcore, math
from utils import LocalLogger


class POI:
    _red: bool = False

    # all poses are relative to the blue field origin
    class Notes:

        class Wing:
            kLeft: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Wing.note_x,
                    constants.FieldPos.Wing.note_init
                ), constants.FieldPos.pose_reverse)

            kRight: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Wing.note_x,
                    constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap
                ), constants.FieldPos.pose_reverse)

            kCenter: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Wing.note_x,
                    constants.FieldPos.Wing.note_init + constants.FieldPos.Wing.note_gap * 2
                ), constants.FieldPos.pose_reverse)

        class MidLine:
            kFar_left: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init
                ), constants.FieldPos.pose_reverse)

            kMid_left: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap
                ), constants.FieldPos.pose_reverse)

            kCenter: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 2
                ), constants.FieldPos.pose_reverse)

            kMid_right: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 3
                ), constants.FieldPos.pose_reverse)

            kFar_right: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.MidLine.mid_line,
                    constants.FieldPos.MidLine.note_init + constants.FieldPos.MidLine.note_gap * 4
                ), constants.FieldPos.pose_reverse)

    class Structures:

        class Scoring:
            kSpeaker: Pose2d = Pose2d(
                Translation2d(
                    0,
                    constants.FieldPos.Scoring.speaker_y
                ), constants.FieldPos.pose_reverse)

            kAmp: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Scoring.amp_x,
                    constants.FieldPos.Scoring.amp_y
                ), constants.FieldPos.Scoring.amp_rotation)

            kAmpActual: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Scoring.amp_x,
                    constants.FieldPos.Scoring.amp_y_robot
                ), constants.FieldPos.Scoring.amp_rotation)

        class Stage:
            center: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Stage.stage_x,
                    constants.FieldPos.Stage.stage_y
                ), constants.FieldPos.pose_reverse)

            left: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.x_deviation,
                    constants.FieldPos.Stage.stage_y + constants.FieldPos.Stage.y_deviation
                ), constants.FieldPos.Stage.left_rotation)

            right: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Stage.stage_x - constants.FieldPos.Stage.x_deviation,
                    constants.FieldPos.Stage.stage_y - constants.FieldPos.Stage.y_deviation
                ), constants.FieldPos.Stage.right_rotation)

        class Pickup:
            source: Pose2d = Pose2d(
                Translation2d(
                    constants.FieldPos.Source.source_x,
                    constants.FieldPos.Source.source_y
                ), constants.FieldPos.Source.rotation)

        class Obstacles:
            pass

        class Waypoints:
            pass

    def __init__(self):
        self._red = False
        self.logger = LocalLogger("POI")
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt.getTable("Odometry")

    def getPoses(self, Object: object):
        poses: list[Pose2d] = list(
            {k: v for k, v in Object.__dict__.items() if isinstance(v, Pose2d)}.values()
        )

        var_names: list[str] = list(
            {k: v for k, v in Object.__dict__.items() if isinstance(v, Pose2d)}.keys()
        )

        return zip(var_names, poses)

    def setValues(self):

        POI = []

        def append(pose: Pose2d):
            x = pose.translation().X()
            y = pose.translation().Y()
            theta = pose.rotation().radians()
            return [x, y, theta]

        for names, note in self.getPoses(self.Notes.Wing):
            POI += append(note)

        for names, note in self.getPoses(self.Notes.MidLine):
            POI += append(note)

        for names, structure in self.getPoses(self.Structures.Scoring):
            POI += append(structure)

        for names, stage in self.getPoses(self.Structures.Stage):
            POI += append(stage)

        for names, pickup in self.getPoses(self.Structures.Pickup):
            POI += append(pickup)

        for names, obstacle in self.getPoses(self.Structures.Obstacles):
            POI += append(obstacle)

        for names, waypoint in self.getPoses(self.Structures.Waypoints):
            POI += append(waypoint)

        self.table.putNumberArray("POI", POI)

    def init(self):

        self.setValues()

    def invertY(self, pose: Pose2d):
        invert = constants.field_width - pose.translation().Y()

        new_rotation = Rotation2d(-pose.rotation().radians())

        new_pose = Pose2d(Translation2d(pose.translation().X(), invert), new_rotation)
        return new_pose

    def setPOI(self, object: object, name: str, pose: Pose2d):
        self.logger.info(f"inverting {name}")
        setattr(object, name, self.invertY(pose))

    def invert(self):
        for name, note in self.getPoses(self.Notes.Wing):
            self.setPOI(self.Notes.Wing, name, note)

        for name, note in self.getPoses(self.Notes.MidLine):
            self.setPOI(self.Notes.MidLine, name, note)

        for name, structure in self.getPoses(self.Structures.Scoring):
            self.setPOI(self.Structures.Scoring, name, structure)

        for name, stage in self.getPoses(self.Structures.Stage):
            self.setPOI(self.Structures.Stage, name, stage)

        for name, pickup in self.getPoses(self.Structures.Pickup):
            self.setPOI(self.Structures.Pickup, name, pickup)

        for name, obstacle in self.getPoses(self.Structures.Obstacles):
            self.setPOI(self.Structures.Obstacles, name, obstacle)

        for name, waypoint in self.getPoses(self.Structures.Waypoints):
            self.setPOI(self.Structures.Waypoints, name, waypoint)

        self.logger.info("inverted")
        self.setValues()

    def setRed(self):
        if self._red:
            return
        self.logger.info("setting red")
        self.invert()
        self._red = True

    def setBlue(self):
        if not self._red:
            return
        self.logger.info("setting blue")
        self.invert()
        self._red = False
