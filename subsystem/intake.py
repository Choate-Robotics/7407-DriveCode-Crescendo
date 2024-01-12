import config
from toolkit.subsystem import Subsystem
from toolkit.motors.revmotor import SparkMax, SparkMaxConfig

INNER_CONFIG = SparkMaxConfig(.5, 0, 0)
OUTER_CONFIG = SparkMaxConfig(.5, 0, 0)

class Intake(Subsystem):
    def __init__(self):
        super().__init__()

        self.inner_motor: SparkMax = SparkMax(
            can_id=config.inner_intake_id
            config=INNER_CONFIG
        )

        self.outer_motor_front: SparkMax = SparkMax(
            can_id=config.outer_intake_front_id
            config=OUTER_CONFIG
        )

        self.outer_motor_back: SparkMax = SparkMax(
            can_id=config.outer_intake_back_id
            config=outer
        )
    
    def init(self):
        self.inner_motor.init()
        self.outer_motor_front.init()
        self.outer_motor_back.init()

    def roll_in(self):
        pass

    def roll_out(self):
        pass

    def deploy_rollers(self):
        pass

    def load_shooter(self):
        pass