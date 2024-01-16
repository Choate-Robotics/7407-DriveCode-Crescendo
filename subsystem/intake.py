import config
import constants
from toolkit.subsystem import Subsystem
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig

INNER_CONFIG = SparkMaxConfig(.5, 0, 0)
OUTER_CONFIG = SparkMaxConfig(.5, 0, 0)

class Intake(Subsystem):
    def __init__(self):
        super().__init__()

        self.inner_motor: SparkMax = SparkMax(
            can_id=config.inner_intake_id,
            config=INNER_CONFIG
        )

        self.outer_motor_front: SparkMax = SparkMax(
            can_id=config.outer_intake_front_id,
            config=OUTER_CONFIG
        )

        self.outer_motor_back: SparkMax = SparkMax(
            can_id=config.outer_intake_back_id,
            config=OUTER_CONFIG
        )
    
    def init(self):
        self.inner_motor.init()
        self.outer_motor_front.init()
        self.outer_motor_back.init()

    def set_inner_velocity(self, vel: float):
        """
        Sets inner motor to given velocity
        param vel: Speed to set motor to in rotations per second (float)
        Return: none
        """
        self.inner_motor.set_target_velocity(vel * constants.intake_inner_gear_ratio)

    def set_outer_velocity(self, vel: float):
        """
        Sets both outer motors to a given velocity
        param vel: Speed to set motors to in rotations per second (float)
        Return: none
        """
        self.outer_motor_front.set_target_velocity(vel * constants.intake_outer_gear_ratio)
        self.outer_motor_back.set_target_velocity(vel * constants.intake_outer_gear_ratio)

    def roll_in(self):
        pass

    def roll_out(self):
        pass

    def deploy_rollers(self):
        pass

    def load_shooter(self):
        pass