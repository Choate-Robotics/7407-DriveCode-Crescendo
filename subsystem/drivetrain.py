import math
from dataclasses import dataclass

import rev
from wpilib import AnalogEncoder
from phoenix6.hardware.cancoder import CANcoder

from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig
from toolkit.motors.ctre_motors import TalonFX, TalonConfig

from toolkit.sensors.gyro import Pigeon2
from toolkit.subsystem_templates.drivetrain import (
    SwerveDrivetrain,
    SwerveNode,
)
from toolkit.utils.units import (
    meters,
    meters_per_second,
    radians,
    radians_per_second,
)
from wpimath.geometry import Pose2d

import config
import constants
from oi.keymap import Keymap
from units.SI import degrees, meters_per_second_squared

TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = TalonConfig(
    0.018, 0.0005, 0.5, 1023 / 22365, neutral_brake=True, integral_zone=1000, max_integral_accumulator=10000
)


@dataclass
class CustomSwerveNode(SwerveNode):
    m_move: TalonFX
    m_turn: SparkMax
    encoder: CANcoder
    absolute_encoder_zeroed_pos: float = 0
    name: str = "DefaultNode"

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    def zero(self):
        current_angle = self.get_motor_angle()

        current_pos_rot = self.encoder.get_absolute_position().value - self.absolute_encoder_zeroed_pos

        self.m_turn.set_sensor_position(current_pos_rot * constants.drivetrain_turn_gear_ratio)

        self.set_motor_angle(current_angle)

    def raw_output(self, power):
        self.m_move.set_raw_output(power)

    def set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def get_motor_angle(self) -> radians:
        return (
                (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
                * 2
                * math.pi
        )

    def set_motor_velocity(self, vel: meters_per_second):
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> radians_per_second:
        return (
                self.m_move.get_sensor_velocity()
                / constants.drivetrain_move_gear_ratio
        )


class Drivetrain(SwerveDrivetrain):
    n_front_left = CustomSwerveNode(
        TalonFX(config.front_left_move_id, config=MOVE_CONFIG),
        SparkMax(config.front_left_turn_id, config=TURN_CONFIG),
        AnalogEncoder(config.front_left_encoder_port),
        absolute_encoder_zeroed_pos=config.front_left_encoder_zeroed_pos,
        name="n_front_left",
    )
    n_front_right = CustomSwerveNode(
        TalonFX(config.front_right_move_id, config=MOVE_CONFIG),
        SparkMax(config.front_right_turn_id, config=TURN_CONFIG),
        AnalogEncoder(config.front_right_encoder_port),
        absolute_encoder_zeroed_pos=config.front_right_encoder_zeroed_pos,
        name="n_front_right",
    )
    n_back_left = CustomSwerveNode(
        TalonFX(config.back_left_move_id, config=MOVE_CONFIG),
        SparkMax(config.back_left_turn_id, config=TURN_CONFIG),
        AnalogEncoder(config.back_left_encoder_port),
        absolute_encoder_zeroed_pos=config.back_left_encoder_zeroed_pos,
        name="n_back_left",
    )
    n_back_right = CustomSwerveNode(
        TalonFX(config.back_right_move_id, config=MOVE_CONFIG),
        SparkMax(config.back_right_turn_id, config=TURN_CONFIG),
        AnalogEncoder(config.back_right_encoder_port),
        absolute_encoder_zeroed_pos=config.back_right_encoder_zeroed_pos,
        name="n_back_right",
    )

    gyro: Pigeon2 = Pigeon2(config.gyro_id)
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    track_width: meters = constants.track_width
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_target_accel: meters_per_second_squared = constants.drivetrain_max_accel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.02
    deadzone_angular_velocity: radians_per_second = math.radians(5)
    start_angle: degrees = 0
    start_pose: Pose2d = Pose2d(
        0,
        0,
        math.radians(start_angle),
    )
    gyro_start_angle: radians = math.radians(start_angle)
    gyro_offset: radians = math.radians(0)

    def x_mode(self):
        self.n_front_left.set_motor_angle(math.radians(-45))
        self.n_front_right.set_motor_angle(math.radians(45))
        self.n_back_left.set_motor_angle(math.radians(45))
        self.n_back_right.set_motor_angle(math.radians(-45))
