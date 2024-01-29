import math
import rev
import config

from toolkit.subsystem import Subsystem
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig
from wpimath.controller import LinearPlantInversionFeedforward_1_1
from wpimath.system.plant import LinearSystemId, DCMotor

FLYWHEEL_CONFIG = SparkMaxConfig(
    0.055, 0.0, 0.01, config.elevator_feed_forward, (-.5, .75), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

class Flywheel(Subsystem):

    def __init__(self):
        super().__init__()

        self.motor_1: SparkMax = SparkMax(
            can_id=config.flywheel_id_1,
            config=FLYWHEEL_CONFIG
        )
        self.motor_2: SparkMax = SparkMax(
            can_id=config.flywheel_id_2,
            config=FLYWHEEL_CONFIG
        )

        self.initialized: bool = False

    @staticmethod
    def rpm_to_angular_velocity(rpm):
        # Convert RPM to radians per second
        angular_velocity = (2 * math.pi * rpm) / 60
        return angular_velocity
    
    @staticmethod
    def angular_velocity_to_rpm(angular_velocity):
        # Convert radians per second to RPM
        rpm = (angular_velocity * 60) / (2 * math.pi)
        return rpm

    def init(self) -> None:
        self.motor_1.init()
        self.motor_2.init()

        self.initialized = True

    def set(self, angular_velocity: float, motor=0) -> None:
        if motor == 1:
            self.motor_1.set_raw_output(self.angular_velocity_to_rpm(angular_velocity))
        elif motor == 2:
            self.motor_2.set_raw_output(self.angular_velocity_to_rpm(angular_velocity))
        else:
            self.motor_1.set_raw_output(self.angular_velocity_to_rpm(angular_velocity))
            self.motor_2.set_raw_output(self.angular_velocity_to_rpm(angular_velocity))

    def get(self, motor=0) -> float:
        if motor == 1:
            return self.rpm_to_angular_velocity(self.motor_1.get_sensor_velocity())
        elif motor == 2:
            return self.rpm_to_angular_velocity(self.motor_2.get_sensor_velocity())
        else:
            return (
                self.rpm_to_angular_velocity(self.motor_1.get_sensor_velocity()),
                self.rpm_to_angular_velocity(self.motor_2.get_sensor_velocity())
            )