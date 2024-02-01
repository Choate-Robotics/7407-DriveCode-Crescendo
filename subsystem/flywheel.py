import math
import rev
import config
import constants

from wpimath.controller import LinearQuadraticRegulator_1_1
from wpimath.estimator import KalmanFilter_1_1_1
from wpimath.system import LinearSystemLoop_1_1_1
from wpimath.system.plant import LinearSystemId, DCMotor

from toolkit.subsystem import Subsystem
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig

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

        self.flywheel_MOI = (constants.flywheel_mass / 2 ) * (constants.flywheel_radius_outer ** 2)

        self.flywheel_plant = LinearSystemId().flywheelSystem(
            DCMotor.NEO(config.flywheel_motor_count), self.flywheel_MOI, constants.flywheel_gear_ratio
        )
        self.flywheel_observer = KalmanFilter_1_1_1(
            self.flywheel_plant,
            [3.0],
            [0.01],
            constants.flywheel_period
        )
        self.flywheel_controller = LinearQuadraticRegulator_1_1(
            self.flywheel_plant,
            [12.0],
            [0.1],
        )
        self.flywheel_controller.latencyCompensate(self.flywheel_plant, constants.flywheel_period, 0.025)
        self.top_flywheel_state = LinearSystemLoop_1_1_1(
            self.flywheel_plant,
            self.flywheel_controller,
            self.flywheel_observer,
            12.0,
            constants.flywheel_period
        )
        self.bottom_flywheel_state = LinearSystemLoop_1_1_1(
            self.flywheel_plant,
            self.flywheel_controller,
            self.flywheel_observer,
            12.0,
            constants.flywheel_period
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

    def set_velocity(self, angular_velocity: float, motor=0) -> None:
        if motor == 1:
            self.motor_1.set_raw_output(self.angular_velocity_to_rpm(angular_velocity))
        elif motor == 2:
            self.motor_2.set_raw_output(self.angular_velocity_to_rpm(angular_velocity))
        else:
            self.motor_1.set_raw_output(self.angular_velocity_to_rpm(angular_velocity))
            self.motor_2.set_raw_output(self.angular_velocity_to_rpm(angular_velocity))

    def get_velocity(self, motor=0) -> float:
        if motor == 1:
            return self.rpm_to_angular_velocity(self.motor_1.get_sensor_velocity())
        elif motor == 2:
            return self.rpm_to_angular_velocity(self.motor_2.get_sensor_velocity())
        else:
            return (
                self.rpm_to_angular_velocity(self.motor_1.get_sensor_velocity()),
                self.rpm_to_angular_velocity(self.motor_2.get_sensor_velocity())
            )
        
    def set_voltage(self, voltage: float, motor=0) -> None:
        if motor == 1:
            self.motor_1.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)
        elif motor == 2:
            self.motor_2.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)
        else:
            self.motor_1.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)
            self.motor_2.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)
    
    def get_voltage(self, motor=0) -> float:
        if motor == 1:
            return self.motor_1.motor.getAppliedOutput()
        elif motor == 2:
            return self.motor_2.motor.getAppliedOutput()
        else:
            return (
                self.motor_1.motor.getAppliedOutput(),
                self.motor_2.motor.getAppliedOutput()
            )
        
    def periodic(self):
        # self.top_flywheel_state.
        # self.bottom_flywheel_state
        ...