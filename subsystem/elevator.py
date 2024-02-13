import rev
import config
import constants

from units.SI import meters

from toolkit.subsystem import Subsystem
from toolkit.motors.rev_motors import SparkMax

class Elevator(Subsystem):

    def __init__(self) -> None:
        super().__init__()
        # Absolute encoder
        self.motor_extend: SparkMax = SparkMax(
            config.elevator_can_id, config=config.ELEVATOR_CONFIG, inverted=False
        )
        self.motor_extend_encoder = None

        self.motor_extend_follower: SparkMax = SparkMax(
            config.elevator_can_id_2, config=config.ELEVATOR_CONFIG, inverted=True
        )

        self.zeroed: bool = False
        self.elevator_moving: bool = False

    def init(self) -> None:
        self.motor_extend.init()
        self.motor_extend_follower.init()

        # Set the motor_extend encoder to the motor's absolute encoder
        self.motor_extend_encoder = self.motor_extend.get_absolute_encoder()

        # Limits motor acceleration
        self.motor_extend.motor.setClosedLoopRampRate(config.elevator_ramp_rate)

        # Inverted b/c motors r parallel facing out.
        self.motor_extend_follower.motor.follow(self.motor_extend.motor, invert=True)
        
        # self.zero()

    @staticmethod
    def length_to_rotations(length: meters) -> float:
        return (length * constants.elevator_gear_ratio) / constants.elevator_driver_gear_circumference
    
    @staticmethod
    def rotations_to_length(rotations: float) -> meters:
        return (rotations * constants.elevator_driver_gear_circumference) / constants.elevator_gear_ratio
    
    @staticmethod
    def limit_length(length: meters) -> meters:
        if length > constants.elevator_max_length:
            return constants.elevator_max_length
        elif length < 0.0:
            return 0.0
        return length
    
    def set_length(self, length: meters) -> None:
        """
        Sets the length of the elevator in meters
        :param length: Length of the elevator (meters)

        """
        length = self.limit_length(length)
        
        self.motor_extend.set_target_position(
            self.length_to_rotations(length)
        )

    def get_length(self) -> float:
        """
        Gets the length of the elevator in meters
        :return: Length of the elevator in meters
        """
        return self.rotations_to_length(self.motor_extend.get_sensor_position())

    def set_motor_extend_position(self, length: meters) -> None:
        """
        Set the position of motor_extend
        :param position: Position of the motor

        """
        length = self.limit_length(length)
        
        self.motor_extend.set_sensor_position(
            self.length_to_rotations(length)
        )

    def zero(self) -> None:
        """
        Zero the elevator

        """
        length = (self.motor_extend_encoder.getPosition() - config.elevator_zeroed_pos) * constants.elevator_max_length
        # Reset the encoder to zero
        self.set_motor_extend_position(length)

        self.zeroed = True

    def set_voltage(self, voltage: float) -> None:
        self.motor_extend.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)

    def get_voltage(self) -> float:
        return self.motor_extend.motor.getAppliedOutput()

    def stop(self) -> None:
        """
        Stop the elevator where it is
        """

        self.set_length(self.get_length())
