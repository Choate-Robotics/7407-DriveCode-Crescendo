import config
import constants
from toolkit.subsystem import Subsystem
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig
from rev import AnalogInput, CANSparkMax


class Intake(Subsystem):
    def __init__(self):
        super().__init__()

        self.inner_motor: SparkMax = SparkMax(
            can_id=config.inner_intake_id,
            config=config.INNER_CONFIG
        )

        self.outer_motor: SparkMax = SparkMax(
            can_id=config.outer_intake_back_id,
            config=config.OUTER_CONFIG,
            brushless=False
        )

        self.deploy_motor: SparkMax = SparkMax(
            can_id=config.deploy_intake_id,
            config=config.DEPLOY_CONFIG
        )

        self.distance_sensor: AnalogInput = None


        self.note_in_intake: bool = False
        self.intake_running: bool = False

        
    
    def init(self):
        self.inner_motor.init()
        self.outer_motor.init()
        self.distance_sensor = self.inner_motor.motor.getAnalog()

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
        self.outer_motor.set_target_velocity(vel * constants.intake_outer_gear_ratio)

    def detect_note(self) -> bool:
        """
        Detects if there is a note in the intake
        Also sets class variable note_in_intake
        Return: true if there is a note, false if there is not
        """
        self.note_in_intake = self.distance_sensor.getVoltage() > config.intake_distance_sensor_threshold
        return self.note_in_intake

    def deploy_roller(self):
        """
        Rotate deploy motor to deploy outer intake
        Return: none
        """
        self.deploy_motor.set_raw_output(0.5)

    def deploy_tenting(self):
        """
        Rotate deploy motor to deploy tenting mechanism
        Return: none
        """
        self.deploy_motor.set_raw_output(-0.5)
    
    def roll_in(self):
        """
        Rolls inner and outer motors in
        Return: none
        """
        self.set_inner_velocity(config.intake_inner_speed)
        self.set_outer_velocity(config.intake_outer_speed)

    def roll_out(self):
        """
        Rolls inner and outer motors out
        Return: none
        """
        self.set_inner_velocity(-config.intake_inner_speed)
        self.set_outer_velocity(-config.intake_outer_speed)

    def rollers_idle_in(self):
        """
        Sets outer motors to their idle speed going in
        Return: none
        """
        self.set_outer_velocity(config.intake_outer_idle_speed)

    def rollers_idle_out(self):
        """
        Sets outer motors to their idle speed going out
        Return: none
        """
        self.set_outer_velocity(-config.intake_outer_idle_speed)
    
    def get_outer_current(self) -> float:
        """
        Return: current of back motor (float)
        """
        return self.outer_motor.motor.getOutputCurrent()
    
    def get_deploy_current(self) -> float:
        """
        Return: current of deploy motor (float)
        """
        return self.deploy_motor.motor.getOutputCurrent()
    
    def roll_inner_in(self):
        """
        Rolls inner motors in
        """
        self.set_inner_velocity(config.intake_inner_speed)
    
    def stop_inner(self):
        """
        Stops inner rollers
        """
        self.set_inner_velocity(0)