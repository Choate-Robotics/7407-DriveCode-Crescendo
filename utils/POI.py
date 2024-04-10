from __future__ import annotations

import math

import ntcore
from wpilib import DriverStation
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Translation2d,
    Translation3d,
)

from units.SI import inches_to_meters

import constants
import config
# from typing import Callable, TypeVar


# from units.SI import feet_to_meters, inches_to_meters, radians
# from utils import LocalLogger


class POIPose:
    _pose: Pose2d | Pose3d
    _red: bool

    # all poses are relative to the blue field origin
    def __init__(self, pose: Pose2d | Pose3d, red_origin: bool = False):
        self._pose = pose
        if not isinstance(self._pose, Pose2d) and not isinstance(self._pose, Pose3d):
            raise TypeError("pose must be Pose2d or Pose3d")
        self._red = red_origin

    def __str__(self):
        return str(self._pose)

    def withRotation(self, rotation: Rotation2d | Rotation3d | int | float):
        """
        sets new rotation for the pose (float and int are interpreted as degrees)

        args:
            rotation: Rotation2d | Rotation3d | int | float - the new rotation in degrees if int or float
        """
        if isinstance(self._pose, Pose3d):
            if isinstance(rotation, float) or isinstance(rotation, int):
                rotation = Rotation3d(0, 0, math.radians(rotation))
            return POIPose(Pose3d(self._pose.translation(), rotation), self._red)
        else:
            if isinstance(rotation, float) or isinstance(rotation, int):
                rotation = Rotation2d(math.radians(rotation))
            return POIPose(Pose2d(self._pose.translation(), rotation), self._red)

    def withOffset(self, offset: Translation2d | Translation3d, red: bool=True) -> POIPose:
        """
        Create a new POIPose with the translation offset by the given amount

        Args:
            offset: Translation2d | Translation3d - the offset to add to the translation

        Returns:
            POIPose: the new POIPose with the translation offset by the given amount
        """
        if red == self._red:
            if isinstance(self._pose, Pose3d):
                return POIPose(
                    Pose3d(self._pose.translation() + offset, self._pose.rotation()),
                    self._red,
                )
            else:
                return POIPose(
                    Pose2d(self._pose.translation() + offset, self._pose.rotation()),
                    self._red,
                )
        else:
            if isinstance(self._pose, Pose3d):
                return POIPose(
                    Pose3d(Translation3d(self._pose.translation().X() + offset.X(),
                                          self._pose.translation().Y() - offset.Y(),
                                           self._pose.translation().Z() + offset.Z()),
                                             self._pose.rotation()),
                    self._red,
                )
            else:
                return POIPose(
                    Pose2d(Translation2d(self._pose.translation().X() + offset.X(),
                                          self._pose.translation().Y() - offset.Y()),
                                            self._pose.rotation()),
                    self._red,
                )

    def __invertY(self, pose: Pose2d | Pose3d) -> Pose2d | Pose3d:
        """
        inverts the y value of the pose

        Args:
            pose: Pose2d | Pose3d - the pose to invert

        Returns:
            POIPose: the new POIPose with the y value inverted
        """
        # TODO This is a weird method because it has nothing to do with the class
        invert = constants.field_width - pose.translation().Y()
        new_rotation: Rotation2d | Rotation3d
        new_pose: Pose2d | Pose3d
        if isinstance(pose, Pose3d):
            new_rotation = Rotation3d(0, 0, -pose.rotation().Z())
            new_pose = Pose3d(
                Translation3d(pose.translation().X(), invert, pose.translation().Z()),
                new_rotation,
            )
        else:
            new_rotation = Rotation2d(-pose.rotation().radians())
            new_pose = Pose2d(
                Translation2d(pose.translation().X(), invert), new_rotation
            )
        return new_pose

    def __check_inversion(self, verbose: bool = False) -> None:
        """
        Checks if a pose is red and inverts it if it is.
        This is a private method and should not be called directly.

        Args:
            verbose: bool - whether or not to print debug information

        Returns:
            None
        """
        # if (
        #     DriverStation.getAlliance() == DriverStation.Alliance.kRed
        #     and not self._red
        #     or DriverStation.getAlliance() is None
        #     and not self._red
        # ):
        #     # print("inverting") if verbose else None
        #     self._pose = self.__invertY(self._pose)
        #     self._red = True
        # elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue and self._red:
        #     # print("inverting") if verbose else None
        #     self._pose = self.__invertY(self._pose)
        #     self._red = False
        # else:
        #     ...
        #     # print("not inverting") if verbose else None

        if (config.active_team == config.Team.RED) and not self._red:
            # print("inverting") if verbose else None
            self._pose = self.__invertY(self._pose)
            self._red = True
        elif (config.active_team == config.Team.BLUE) and self._red:
            # print("inverting") if verbose else None
            self._pose = self.__invertY(self._pose)
            self._red = False
        else:
            ...
            # print("not inverting") if verbose else None

    def get(self, verbose: bool = True) -> Pose2d:
        """
        returns the Pose2d (and only Pose2d), inverted if red is true

        Args:
            verbose: bool - whether or not to print debug information

        Returns:
            Pose2d: the pose, inverted if red is true
        """
        # if red is true, invert the y value
        self.__check_inversion(verbose)
        if isinstance(self._pose, Pose3d):
            return self._pose.toPose2d()
        return self._pose

    def get3d(self, verbose: bool = True) -> Pose3d:
        """
        returns the Pose3d, inverted if red is true

        Args:
            verbose: bool - whether or not to print debug information

        Returns:
            Pose3d: the pose, inverted if red is true
        """
        if not isinstance(self._pose, Pose3d):
            raise TypeError("pose must be Pose3d")
        # if red is true, invert the y value
        self.__check_inversion(verbose)
        return self._pose

    def getZ(self) -> float:
        """
        returns the z value of the pose, 0 if the pose is Pose2d

        Returns:
            float: the z value of the pose, 0 if the pose is Pose2d
        """
        if isinstance(self._pose, Pose3d):
            return self._pose.translation().Z()
        else:
            return 0

    def getTranslation(self) -> Translation2d | Translation3d:
        """
        returns the translation of the pose associated with the POIPose

        Returns:
            Translation2d | Translation3d: the translation of the pose associated with the POIPose
        """
        return self.get().translation()


class POI:
    _red: bool = False

    class Coordinates:
        class Notes:
            class Wing:
                kRight = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Wing.note_x,
                            constants.FieldPos.Wing.note_init,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kCenter = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Wing.note_x,
                            constants.FieldPos.Wing.note_init
                            + constants.FieldPos.Wing.note_gap,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kLeft = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Wing.note_x,
                            constants.FieldPos.Wing.note_init
                            + constants.FieldPos.Wing.note_gap * 2,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

            class MidLine:
                kFarRight = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.MidLine.mid_line,
                            constants.FieldPos.MidLine.note_init,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kMidRight = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.MidLine.mid_line,
                            constants.FieldPos.MidLine.note_init
                            + constants.FieldPos.MidLine.note_gap,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kCenter = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.MidLine.mid_line,
                            constants.FieldPos.MidLine.note_init
                            + constants.FieldPos.MidLine.note_gap * 2,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kMidLeft = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.MidLine.mid_line,
                            constants.FieldPos.MidLine.note_init
                            + constants.FieldPos.MidLine.note_gap * 3,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kFarLeft = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.MidLine.mid_line,
                            constants.FieldPos.MidLine.note_init
                            + constants.FieldPos.MidLine.note_gap * 4,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

        class Structures:
            class Scoring:
                kSpeaker = POIPose(
                    Pose3d(
                        Translation3d(
                            constants.FieldPos.Scoring.speaker_x,
                            constants.FieldPos.Scoring.speaker_y,
                            constants.FieldPos.Scoring.speaker_z
                        ),
                        Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians()),
                    )
                )

                kAmp = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Scoring.amp_x,
                            constants.FieldPos.Scoring.amp_y,
                        ),
                        constants.FieldPos.Scoring.amp_rotation,
                    )
                )

                kAmpActual = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Scoring.amp_x,
                            constants.FieldPos.Scoring.amp_y_robot,
                        ),
                        constants.FieldPos.Scoring.amp_rotation,
                    )
                )

            class Stage:
                kCenter = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Stage.stage_x,
                            constants.FieldPos.Stage.stage_y,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kLeft = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.x_deviation,
                            constants.FieldPos.Stage.stage_y
                            + constants.FieldPos.Stage.y_deviation,
                        ),
                        constants.FieldPos.Stage.left_rotation,
                    )
                )

                kRight = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.x_deviation,
                            constants.FieldPos.Stage.stage_y
                            - constants.FieldPos.Stage.y_deviation,
                        ),
                        constants.FieldPos.Stage.right_rotation,
                    )
                )

            class Pickup:
                source = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Source.source_x,
                            constants.FieldPos.Source.source_y,
                        ),
                        constants.FieldPos.Source.rotation,
                    )
                )

            class Obstacles:
                # Obstacles are usually in Pose3d, with the z acting
                # as the recommended minimum distance to avoid the obstacle

                kStage = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.stage_length / 2
                            + constants.FieldPos.Stage.post_deviation,
                            constants.FieldPos.Stage.stage_y,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kStageCenterPost = POIPose(
                    Pose3d(
                        Translation3d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.stage_length
                            + constants.FieldPos.Stage.post_deviation,
                            constants.FieldPos.Stage.stage_y,
                            constants.post_avoidance_distance,
                        ),
                        Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians()),
                    )
                )

                kStageLeftPost = POIPose(
                    Pose3d(
                        Translation3d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.post_deviation,
                            constants.FieldPos.Stage.stage_y
                            + constants.FieldPos.Stage.stage_width / 2
                            - constants.FieldPos.Stage.post_deviation,
                            constants.post_avoidance_distance,
                        ),
                        Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians()),
                    )
                )

                kStageRightPost = POIPose(
                    Pose3d(
                        Translation3d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.post_deviation,
                            constants.FieldPos.Stage.stage_y
                            - constants.FieldPos.Stage.stage_width / 2
                            + constants.FieldPos.Stage.post_deviation,
                            constants.post_avoidance_distance,
                        ),
                        Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians()),
                    )
                )

                kStage = POIPose(
                    Pose2d(
                        Translation2d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.stage_length / 2
                            + constants.FieldPos.Stage.post_deviation,
                            constants.FieldPos.Stage.stage_y,
                        ),
                        constants.FieldPos.pose_reverse,
                    )
                )

                kStageCenterPost = POIPose(
                    Pose3d(
                        Translation3d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.stage_length
                            + constants.FieldPos.Stage.post_deviation,
                            constants.FieldPos.Stage.stage_y,
                            constants.post_avoidance_distance,
                        ),
                        Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians()),
                    )
                )

                kStageLeftPost = POIPose(
                    Pose3d(
                        Translation3d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.post_deviation,
                            constants.FieldPos.Stage.stage_y
                            + constants.FieldPos.Stage.stage_width / 2
                            - constants.FieldPos.Stage.post_deviation,
                            constants.post_avoidance_distance,
                        ),
                        Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians()),
                    )
                )

                kStageRightPost = POIPose(
                    Pose3d(
                        Translation3d(
                            constants.FieldPos.Stage.stage_x
                            - constants.FieldPos.Stage.post_deviation,
                            constants.FieldPos.Stage.stage_y
                            - constants.FieldPos.Stage.stage_width / 2
                            + constants.FieldPos.Stage.post_deviation,
                            constants.post_avoidance_distance,
                        ),
                        Rotation3d(0, 0, constants.FieldPos.pose_reverse.radians()),
                    )
                )

        class Waypoints:
            class Auto:
                kMidlineAutoScoring = POIPose(
                    Pose2d(
                        Translation2d(
                            3.72,
                            2.55
                        ),
                        Rotation2d(constants.FieldPos.pose_reverse.radians())
                    )
                )
                kMidlineAutoStart = POIPose(
                    Pose2d(
                        Translation2d(
                            1.9 - constants.drivetrain_length_with_bumpers/2,
                            2.92
                        ),
                        Rotation2d(constants.FieldPos.pose_reverse.radians())
                    )
                )
                kSubwooferRight = POIPose(
                    Pose2d(
                        Translation2d(
                            0.67,
                            constants.field_width/2 + 0.3
                        ),
                        Rotation2d(math.radians(120))
                    )
                )
            

    def __init__(self):
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Odometry")
        self.table = self.nt.getSubTable("POI")
        self.setNTValues()

    def setNTValues(self):
        # set NT values for all coordinates dynamically, as a list

        # get all classes in Coordinates
        classes = [
            cls for cls in self.Coordinates.__dict__.values() if isinstance(cls, type)
        ]
        # find variables in all classes that are POIPose

        for cls in classes:
            table = self.table.getSubTable(cls.__name__)
            for classvar in cls.__dict__.values():
                list_of_POIPoses = []
                if isinstance(classvar, type):
                    for var in classvar.__dict__.values():
                        if isinstance(var, POIPose):
                            pose = var.get(False)
                            list_of_POIPoses += [
                                pose.X(),
                                pose.Y(),
                                pose.rotation().radians(),
                            ]

                    table.putNumberArray(classvar.__name__, list_of_POIPoses)


def within_point_distance(poses: list[Pose2d], point: Pose2d, distance: float) -> bool:
    """
    checks if a point is within a certain distance of any of the poses

    Args:
        poses: list[Pose2d] - the list of poses to check
        point: Pose2d - the point to check
        distance: float - the distance to check

    Returns:
        bool: whether or not the point is within the distance of any of the poses
    """
    for pose in poses:
        if pose.translation().distance(point.translation()) < distance:
            return True
    return False
