import math
import rev
import config
import constants

from wpimath.controller import LinearQuadraticRegulator_1_1
from wpimath.estimator import KalmanFilter_1_1_1
from wpimath.system import LinearSystemLoop_1_1_1
from wpimath.system.plant import LinearSystemId, DCMotor

from toolkit.subsystem import Subsystem
from toolkit.motors.rev_motors import SparkMax

from units.SI import radians_per_second, meters_per_second, radians

class Flywheel(Subsystem):

    def __init__(self):
        super().__init__()

        self.motor_1: SparkMax = SparkMax(
            can_id=config.flywheel_id_1,
            config=config.FLYWHEEL_CONFIG
        )
        self.motor_2: SparkMax = SparkMax(
            can_id=config.flywheel_id_2,
            config=config.FLYWHEEL_CONFIG
        )

        self.flywheel_MOI = (constants.flywheel_mass / 2 ) * (constants.flywheel_radius_outer ** 2)

        self.flywheel_plant = LinearSystemId().flywheelSystem(
            DCMotor.NEO(config.flywheel_motor_count), self.flywheel_MOI, constants.flywheel_gear_ratio
        )
        self.flywheel_observer = KalmanFilter_1_1_1(
            self.flywheel_plant,
            [3.0], # how accurate we think our model is
            [0.01], # how accurate we think our encoder data is
            config.period
        )
        self.flywheel_controller = LinearQuadraticRegulator_1_1(
            self.flywheel_plant,
            [8.0], # velocity error tolerance
            [12.0], # control effort tolerance
            config.period
        )
        self.flywheel_controller.latencyCompensate(self.flywheel_plant, config.period, 0.025)
        self.top_flywheel_state = LinearSystemLoop_1_1_1(
            self.flywheel_plant,
            self.flywheel_controller,
            self.flywheel_observer,
            12.0,
            config.period
        )
        self.bottom_flywheel_state = LinearSystemLoop_1_1_1(
            self.flywheel_plant,
            self.flywheel_controller,
            self.flywheel_observer,
            12.0,
            config.period
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
    
    @staticmethod
    def linear_velocity_to_angular_velocity(linear_velocity):
        # Convert radians per second to RPM
        vel = linear_velocity / constants.flywheel_radius_outer
        return vel
    
    @staticmethod
    def angular_velocity_to_linear_velocity(angular_velocity):
        # Convert radians per second to RPM
        vel = angular_velocity * constants.flywheel_radius_outer
        return vel

    def init(self) -> None:
        self.motor_1.init()
        self.motor_2.init()

        self.initialized = True

    def set_velocity(self, angular_velocity: radians_per_second, motor=0) -> None:
        if motor == 1:
            self.top_flywheel_state.setNextR([angular_velocity])
        elif motor == 2:
            self.bottom_flywheel_state.setNextR([angular_velocity])
        else:
            self.top_flywheel_state.setNextR([angular_velocity])
            self.bottom_flywheel_state.setNextR([angular_velocity])
            
    def set_velocity_linear(self, linear_velocity: meters_per_second, motor=0) -> None:
        angular_velocity = linear_velocity / constants.flywheel_radius_outer
        self.set_velocity(angular_velocity, motor)


    def get_velocity(self, motor=0) -> radians_per_second:
        if motor == 1:
            return self.rpm_to_angular_velocity(self.motor_1.get_sensor_velocity())
        elif motor == 2:
            return self.rpm_to_angular_velocity(self.motor_2.get_sensor_velocity())
        else:
            return (
                self.rpm_to_angular_velocity(self.motor_1.get_sensor_velocity()),
                self.rpm_to_angular_velocity(self.motor_2.get_sensor_velocity())
            )
            
    def get_velocity_linear(self, motor=0) -> meters_per_second:
        return self.get_velocity(motor) * constants.flywheel_radius_outer
        
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
            
    def within_velocity(self, velocity: radians_per_second, tolerance: radians_per_second, motor=0) -> bool:
        '''
        Returns True if the flywheel velocity is within the tolerance of the target velocity
        '''
        
        def tol(vel, target, tol):
            return abs(vel - target) < tol
        
        if motor == 1:
            return tol(self.get_velocity(1), velocity, tolerance)
        elif motor == 2:
            return tol(self.get_velocity(2), velocity, tolerance)
        else:
            return (
                tol(self.get_velocity(1), velocity, tolerance) and
                tol(self.get_velocity(2), velocity, tolerance)
            )
    
    def within_velocity_linear(self, velocity: meters_per_second, tolerance: meters_per_second, motor=0) -> bool:
        '''
        Returns True if the flywheel velocity is within the tolerance of the target velocity
        '''
        
        def tol(vel, target, tol):
            return abs(vel - target) < tol
        
        if motor == 1:
            return tol(self.get_velocity_linear(1), velocity, tolerance)
        elif motor == 2:
            return tol(self.get_velocity_linear(2), velocity, tolerance)
        else:
            return (
                tol(self.get_velocity_linear(1), velocity, tolerance) and
                tol(self.get_velocity_linear(2), velocity, tolerance)
            )
        
    def periodic(self):
        
        # Correct the state estimate with the encoder and voltage
        self.top_flywheel_state.correct([self.get_velocity(1)])
        self.bottom_flywheel_state.correct([self.get_velocity(2)])

        # Update our LQR to generate new voltage commands and use the voltage
        self.top_flywheel_state.predict(config.period)
        self.bottom_flywheel_state.predict(config.period)

        # Set the next setpoint for the flywheel
        self.set_voltage(self.top_flywheel_state.U(0), 1)
        self.set_voltage(self.bottom_flywheel_state.U(0), 2)
