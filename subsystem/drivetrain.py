from __future__ import annotations

import math
import config
import constants
import ntcore

from dataclasses import dataclass
from wpilib import AnalogEncoder
from wpimath.geometry import Pose2d
from units.SI import (

    meters,
    meters_per_second,
    radians,
    radians_per_second,
    meters_per_second_squared,
    degrees
)

from oi.keymap import Keymap
from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig
from toolkit.motors.ctre_motors import TalonFX
from toolkit.sensors.gyro import Pigeon2
from toolkit.subsystem_templates.drivetrain import (
    SwerveDrivetrain,
    SwerveNode,
)
foc_active = False


@dataclass
class CustomSwerveNode(SwerveNode):
    m_move: TalonFX
    m_turn: SparkMax
    encoder: AnalogEncoder
    absolute_encoder_zeroed_pos: float = 0
    name: str = "DefaultNode"
    counter: int = 0

    def init(self):
        print(f"Initializing {self.name}", self.counter)
        self.m_move.init()
        self.m_turn.init()
        self.m_turn.optimize_normal_sparkmax()
        self.counter += 1

    def zero(self):
        """
        Zeros the drivetrain

        """

        self.m_turn.set_sensor_position(0)
        abs_encoder_position: float = self.encoder.getAbsolutePosition()

        # Find the difference in current and zero absolute position
        encoder_difference: float = abs_encoder_position - self.absolute_encoder_zeroed_pos

        if encoder_difference > .5:
            encoder_difference -= 1
        elif encoder_difference < -.5:
            encoder_difference += 1

        motor_change = encoder_difference * constants.drivetrain_turn_gear_ratio
        
        ntcore.NetworkTableInstance.getDefault().getTable('swerve').putNumber('where the motor thinks it should be', motor_change)

        self.m_turn.set_sensor_position(motor_change)

        self.m_turn.set_target_position(0)

    def get_abs(self):
        return self.encoder.getAbsolutePosition()

    def set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def get_turn_motor_angle(self) -> radians:
        return (
                (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
                * 2
                * math.pi
        )

    def set_motor_velocity(self, vel: meters_per_second):
        rotations_per_second = (
                vel *
                constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )
        
        print(rotations_per_second)

        self.m_move.set_target_velocity(rotations_per_second)

    def get_motor_velocity(self) -> meters_per_second:
        return (
                self.m_move.get_sensor_velocity()
                / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    def get_drive_motor_traveled_distance(self) -> meters:
        sensor_position = self.m_move.get_sensor_position()
        return (
                sensor_position
                / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )


class Drivetrain(SwerveDrivetrain):
    n_front_left = CustomSwerveNode(
        TalonFX(config.front_left_move_id, foc=foc_active, config=config.MOVE_CONFIG),
        SparkMax(config.front_left_turn_id, config=config.TURN_CONFIG, inverted=True),
        config.front_left_encoder_port,
        absolute_encoder_zeroed_pos=config.front_left_encoder_zeroed_pos,
        name="n_front_left",
    )
    n_front_right = CustomSwerveNode(
        TalonFX(config.front_right_move_id, foc=foc_active, config=config.MOVE_CONFIG, inverted=True),
        SparkMax(config.front_right_turn_id, config=config.TURN_CONFIG, inverted=False),
        config.front_right_encoder_port,
        absolute_encoder_zeroed_pos=config.front_right_encoder_zeroed_pos,
        name="n_front_right",
    )
    n_back_left = CustomSwerveNode(
        TalonFX(config.back_left_move_id, foc=foc_active, config=config.MOVE_CONFIG, inverted=True),
        SparkMax(config.back_left_turn_id, config=config.TURN_CONFIG, inverted=False),
        config.back_left_encoder_port,
        absolute_encoder_zeroed_pos=config.back_left_encoder_zeroed_pos,
        name="n_back_left",
    )
    n_back_right = CustomSwerveNode(
        TalonFX(config.back_right_move_id, foc=foc_active, config=config.MOVE_CONFIG),
        SparkMax(config.back_right_turn_id, config=config.TURN_CONFIG, inverted=True),
        config.back_right_encoder_port,
        absolute_encoder_zeroed_pos=config.back_right_encoder_zeroed_pos,
        name="n_back_right",
    )

    gyro: Pigeon2 = Pigeon2(config.gyro_id)
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    track_width: meters = constants.track_width
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.05
    deadzone_angular_velocity: radians_per_second = math.radians(5)
    start_angle: degrees = 0
    start_pose: Pose2d = Pose2d(
        0,
        0,
        math.radians(start_angle),
    )
    gyro_start_angle: radians = math.radians(start_angle)
    gyro_offset: radians = math.radians(0)
    ready_to_shoot: bool = False

    def x_mode(self):
        self.n_front_left.set_motor_angle(math.radians(-45))
        self.n_front_right.set_motor_angle(math.radians(45))
        self.n_back_left.set_motor_angle(math.radians(45))
        self.n_back_right.set_motor_angle(math.radians(-45))

    def get_abs(self):
        fl = self.n_front_left.get_abs() #0.467
        fr = self.n_front_right.get_abs() #0.060
        bl = self.n_back_left.get_abs() #0.727
        br = self.n_back_right.get_abs() #0.860
        return [fl, fr, bl, br]
    
