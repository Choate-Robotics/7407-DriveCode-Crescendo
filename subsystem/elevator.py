import rev
import config
import constants

from units.SI import meters
import ntcore
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
        self.locked: bool = False
        self.target_length: meters = 0.0

    def init(self) -> None:
        self.motor_extend.init()
        self.motor_extend_follower.init()

        # Set the motor_extend encoder to the motor's absolute encoder
        self.motor_extend_encoder = self.motor_extend_follower.get_absolute_encoder()

        # self.motor_extend_follower.motor.follow(self.motor_extend.motor, invert=True)
        self.motor_extend_follower.follow(self.motor_extend, True)
        # self.motor_extend_follower.motor.burnFlash()

        # Limits motor acceleration
        self.motor_extend.motor.setClosedLoopRampRate(config.elevator_ramp_rate)

        # Inverted b/c motors r parallel facing out.

        # self.zero()

    @staticmethod
    def length_to_rotations(length: meters) -> float:
        return (length * constants.elevator_gear_ratio) / constants.elevator_driver_gear_circumference

    @staticmethod
    def rotations_to_length(rotations: float) -> meters:
        return (rotations * constants.elevator_driver_gear_circumference) / constants.elevator_gear_ratio

    def limit_length(self, length: meters) -> meters:
        if self.locked and length > constants.elevator_max_length_stage:
            return constants.elevator_max_length_stage
        if length > constants.elevator_max_length:
            return constants.elevator_max_length
        elif length < 0.0:
            return 0.0
        return length

    def set_length(self, length: meters, arbff: float = 0) -> None:
        """
        Sets the length of the elevator in meters
        :param length: Length of the elevator (meters)
        :param arbff: feed forward for the elevator
        """
        length = self.limit_length(length)
        self.target_length = length

        print(length)
        print(self.length_to_rotations(length), 'elevator rotation')

        self.motor_extend.set_target_position(
            self.length_to_rotations(length), arbff
        )

    def get_length(self) -> float:
        """
        Gets the length of the elevator in meters
        :return: Length of the elevator in meters
        """
        return self.rotations_to_length(self.motor_extend.get_sensor_position())

    def get_length_total_height(self) -> meters:

        return self.get_length() + constants.elevator_bottom_total_height

    def set_motor_extend_position(self, length: meters) -> None:
        """
        Set the position of motor_extend
        :param position: Position of the motor

        """
        length = self.limit_length(length)

        self.motor_extend.set_sensor_position(
            self.length_to_rotations(length)
        )

    def get_elevator_abs(self) -> meters:

        length = (self.motor_extend_encoder.getPosition() - config.elevator_zeroed_pos) * constants.elevator_max_length
        length = 0 if length < 0 else length
        # print(length, 'abs length')
        return length

    def zero(self) -> None:
        """
        Zero the elevator

        """

        length = self.limit_length(self.get_elevator_abs())

        print(length, 'elevator length (m)')
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

    def lock(self) -> None:

        self.locked = True

    def unlock(self) -> None:
        self.locked = False

    def periodic(self) -> None:

        table = ntcore.NetworkTableInstance.getDefault().getTable('elevator')

        table.putNumber('elevator height', self.get_length())
        table.putNumber('elevator abs height', self.get_elevator_abs())
        table.putBoolean('elevator moving', self.elevator_moving)
        table.putBoolean('elevator locked', self.locked)
        table.putBoolean('elevator zeroed', self.zeroed)
        table.putNumber('elevator height total', self.get_length_total_height())
        table.putNumber('elevator target height', self.target_length)
        table.putNumber('elevator motor lead applied output', self.motor_extend.motor.getAppliedOutput())
        table.putNumber('elevator motor follow applied output', self.motor_extend_follower.motor.getAppliedOutput())
