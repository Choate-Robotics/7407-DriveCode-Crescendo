from wpimath.geometry import Pose3d


class VisionEstimator:
    """
    An estimator (e.g. limelight, photon-vision) that returns a list of robot poses relative to the field.
    """

    def __init__(self):
        pass

    def get_estimated_robot_pose(self) -> list[tuple[tuple[Pose3d, float], Pose3d]] | None:
        """
        Returns the robot's pose relative to the field, estimated by the vision system. Override this method.
        :return: Vision system estimate of robot pose along with the associated timestamp.
        :rtype: list[Pose3d, seconds: float] | None
        """
        raise NotImplementedError
