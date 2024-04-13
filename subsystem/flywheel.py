import math


import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
from units.SI import meters_per_second, radians_per_second
from utils import NetworkTable

foc_active = False


class Flywheel(Subsystem):
    def __init__(self):
        super().__init__()



        self.motor_1: TalonFX = TalonFX(
            can_id=config.flywheel_id_1,
            foc=foc_active,
            inverted=True,
            config=config.FLYWHEEL_CONFIG,
        )
        self.motor_2: TalonFX = TalonFX(
            can_id=config.flywheel_id_2,
            foc=foc_active,
            inverted=True,
            config=config.FLYWHEEL_CONFIG,
        )

        # self.motor_1.set_average_depth()
        self.flywheel_top_target = 0
        self.flywheel_bottom_target = 0

        self.ready_to_shoot: bool = False
        self.initialized: bool = False

    @staticmethod
    def rpm_to_angular_velocity(rpm):
        # Convert RPM to radians per second
        angular_velocity = (2 * math.pi * rpm) / 60
        return angular_velocity

    @staticmethod
    def rps_to_angular_velocity(rps):
        # Convert RPM to radians per second
        angular_velocity = 2 * math.pi * rps
        return angular_velocity

    @staticmethod
    def angular_velocity_to_rpm(angular_velocity):
        # Convert radians per second to RPM
        rpm = (angular_velocity * 60) / (2 * math.pi)
        return rpm

    @staticmethod
    def angular_velocity_to_rps(angular_velocity):
        # Convert radians per second to RPS
        rps = (angular_velocity) / (2 * math.pi)
        return rps

    @staticmethod
    def linear_velocity_to_angular_velocity(linear_velocity):
        # convert linear to angular velocity
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

    def note_shot(self) -> bool:
        return (
            self.get_current(1) > config.flywheel_shot_current_threshold
            or self.get_current(2) > config.flywheel_shot_current_threshold
        )

    def set_velocity(self, angular_velocity: radians_per_second, motor=0) -> None:
        if motor == 1:
            # self.top_flywheel_state.setNextR([angular_velocity])
            self.flywheel_top_target = angular_velocity
            self.motor_1.set_target_velocity(
                self.angular_velocity_to_rps(angular_velocity), 0
            )
        elif motor == 2:
            # self.bottom_flywheel_state.setNextR([angular_velocity])
            self.flywheel_bottom_target = angular_velocity
            self.motor_2.set_target_velocity(
                self.angular_velocity_to_rps(angular_velocity), 0
            )
        else:
            # self.top_flywheel_state.setNextR([angular_velocity])
            self.flywheel_top_target = angular_velocity
            self.flywheel_bottom_target = angular_velocity
            self.motor_1.set_target_velocity(
                self.angular_velocity_to_rps(angular_velocity), 0
            )

            # self.bottom_flywheel_state.setNextR([angular_velocity])
            self.motor_2.set_target_velocity(
                self.angular_velocity_to_rps(angular_velocity), 0
            )

    def set_velocity_linear(self, linear_velocity: meters_per_second, motor=0) -> None:
        angular_velocity = linear_velocity / constants.flywheel_radius_outer
        self.set_velocity(angular_velocity, motor)

    def get_velocity(self, motor=0) -> radians_per_second:
        if motor == 1:
            return self.rps_to_angular_velocity(self.motor_1.get_sensor_velocity())
        elif motor == 2:
            return self.rps_to_angular_velocity(self.motor_2.get_sensor_velocity())
        else:
            return (
                self.rps_to_angular_velocity(self.motor_1.get_sensor_velocity()),
                self.rps_to_angular_velocity(self.motor_2.get_sensor_velocity()),
            )

    def get_velocity_linear(self, motor=0) -> meters_per_second:
        if motor == 0:
            return (
                (self.get_velocity(1) + self.get_velocity(2))
                / 2
                * constants.flywheel_radius_outer
            )
        else:
            return self.get_velocity(motor) * constants.flywheel_radius_outer


    def get_current(self, motor=0) -> float:
        if motor == 1:
            return self.motor_1.get_motor_current()
        elif motor == 2:
            return self.motor_2.get_motor_current()
        else:
            return (
                self.motor_1.get_motor_current(),
                self.motor_2.get_motor_current(),
            )

    def within_velocity(
        self, velocity: radians_per_second, tolerance: radians_per_second, motor=0
    ) -> bool:
        """
        Returns True if the flywheel velocity is within the tolerance of the target velocity
        """

        def tol(vel, target, tol):
            return abs(vel - target) < tol

        if motor == 1:
            return tol(self.get_velocity(1), velocity, tolerance)
        elif motor == 2:
            return tol(self.get_velocity(2), velocity, tolerance)
        else:
            return tol(self.get_velocity(1), velocity, tolerance) and tol(
                self.get_velocity(2), velocity, tolerance
            )

    def within_velocity_linear(
        self, velocity: meters_per_second, tolerance: meters_per_second, motor=0
    ) -> bool:
        """
        Returns True if the flywheel velocity is within the tolerance of the target velocity
        """

        def tol(vel, target, tol):
            return abs(vel - target) < tol

        if motor == 1:
            return tol(self.get_velocity_linear(1), velocity, tolerance)
        elif motor == 2:
            return tol(self.get_velocity_linear(2), velocity, tolerance)
        else:
            return tol(self.get_velocity_linear(1), velocity, tolerance) and tol(
                self.get_velocity_linear(2), velocity, tolerance
            )

    def periodic(self):
        # # Correct the state estimate with the encoder and voltage
        # self.top_flywheel_state.correct([self.get_velocity(1)])
        # self.bottom_flywheel_state.correct([self.get_velocity(2)])

        # # Update our LQR to generate new voltage commands and use the voltage
        # self.top_flywheel_state.predict(config.period)
        # self.bottom_flywheel_state.predict(config.period)

        # # # Set the next setpoint for the flywheel
        # self.set_voltage(self.top_flywheel_state.U(0), 1)
        # self.set_voltage(self.bottom_flywheel_state.U(0), 2)

        if self.within_velocity_linear(
            self.angular_velocity_to_linear_velocity(self.flywheel_top_target),
            config.flywheel_shot_tolerance,
        ):
            self.ready_to_shoot = True
        else:
            self.ready_to_shoot = False

        # table = ntcore.NetworkTableInstance.getDefault().getTable("flywheel")
        table = NetworkTable("flywheel")
        table.putNumber("flywheel top velocity", self.get_velocity_linear(1))
        table.putNumber("flywheel bottom velocity", self.get_velocity_linear(2))
        table.putBoolean("ready to shoot", self.ready_to_shoot)
        table.putBoolean("note shot", self.note_shot())
        table.putNumber("flywheel top velocity rpm", self.motor_1.get_sensor_velocity())
        table.putNumber(
            "flywheel top target",
            self.angular_velocity_to_linear_velocity(self.flywheel_top_target),
        )
        table.putNumber(
            "flywheel bottom target",
            self.angular_velocity_to_linear_velocity(self.flywheel_bottom_target),
        )
        # table.putNumber("flywheel top voltage", self.get_voltage(1))
        # table.putNumber("flywheel bottom voltage", self.get_voltage(2))
        # table.putNumber("flywheel top current", self.get_current(1))
        # table.putNumber("flywheel bottom current", self.get_current(2))
        # table.putNumber("flywheel top bus voltage", self.motor_1.motor.getBusVoltage())
        # table.putNumber(
        #     "flywheel bottom bus voltage", self.motor_2.motor.getBusVoltage()
        # )
        # table.putNumber(
        #     "flywheel top applied output", self.motor_1.motor.getAppliedOutput()
        # )
        # table.putNumber(
        #     "flywheel bottom applied output", self.motor_2.motor.getAppliedOutput()
        # )
        # table.putNumber(
        #     "flywheel average depth top motor", self.motor_1.encoder.getAverageDepth()
        # )
        # table.putNumber(
        #     "flywheel measurement period top motor", self.motor_1.encoder.getMeasurementPeriod()
        # )
