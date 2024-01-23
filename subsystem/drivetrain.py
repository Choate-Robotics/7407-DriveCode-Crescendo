from __future__ import annotations

import math
from dataclasses import dataclass

import rev
from wpilib import AnalogEncoder

from toolkit.motors.rev_motors import SparkMax, SparkMaxConfig
from toolkit.motors.ctre_motors import TalonFX, TalonConfig

from toolkit.sensors.gyro import Pigeon2
from toolkit.subsystem_templates.drivetrain import (
    SwerveDrivetrain,
    SwerveNode,
)
from units.SI import (
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
    0.11, 0, 0, 0.25, 0.01, brake_mode=True  # integral_zone=1000, max_integral_accumulator=10000
)


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
        self.counter += 1

    def initial_zero(self):
        self.m_turn.set_sensor_position(0)
        abs_encoder_position: float = self.encoder.getAbsolutePosition()
        print(abs_encoder_position)

        encoder_difference: float = abs_encoder_position - self.absolute_encoder_zeroed_pos

        if encoder_difference > .5:
            encoder_difference -= 1
        elif encoder_difference < -.5:
            encoder_difference += 1

        motor_change = encoder_difference * constants.drivetrain_turn_gear_ratio

        print(-encoder_difference * 360, self.m_turn._can_id)
        self.m_turn.set_sensor_position(motor_change)

    def zero(self):
        self.initial_zero()

        self.m_turn.set_target_position(0)

    def get_abs(self):
        return self.encoder.getAbsolutePosition()

    def raw_output(self, power):
        self.m_move.set_raw_output(power)

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
        # print(vel, 'meters per second')
        rotations_per_second = vel * constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        # print(rotations_per_second, 'rotations per second')
        self.m_move.set_target_velocity(rotations_per_second)

    def get_motor_velocity(self) -> meters_per_second:
        return (
                self.m_move.get_sensor_velocity()
                / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    def get_drive_motor_traveled_distance(self) -> meters:

        sensor_position = self.m_move.get_sensor_position()

        return (
                sensor_position / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )


foc_active = False


class Drivetrain(SwerveDrivetrain):
    n_front_left = CustomSwerveNode(
        TalonFX(config.front_left_move_id, foc=foc_active, config=MOVE_CONFIG),
        SparkMax(config.front_left_turn_id, config=TURN_CONFIG),
        config.front_left_encoder_port,
        absolute_encoder_zeroed_pos=config.front_left_encoder_zeroed_pos,
        name="n_front_left",
    )
    n_front_right = CustomSwerveNode(
        TalonFX(config.front_right_move_id, foc=foc_active, config=MOVE_CONFIG),
        SparkMax(config.front_right_turn_id, config=TURN_CONFIG),
        config.front_right_encoder_port,
        absolute_encoder_zeroed_pos=config.front_right_encoder_zeroed_pos,
        name="n_front_right",
    )
    n_back_left = CustomSwerveNode(
        TalonFX(config.back_left_move_id, foc=foc_active, config=MOVE_CONFIG),
        SparkMax(config.back_left_turn_id, config=TURN_CONFIG),
        config.back_left_encoder_port,
        absolute_encoder_zeroed_pos=config.back_left_encoder_zeroed_pos,
        name="n_back_left",
    )
    n_back_right = CustomSwerveNode(
        TalonFX(config.back_right_move_id, foc=foc_active, config=MOVE_CONFIG),
        SparkMax(config.back_right_turn_id, config=TURN_CONFIG),
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

    def get_abs(self):
        fl = self.n_front_left.get_abs()
        fr = self.n_front_right.get_abs()
        bl = self.n_back_left.get_abs()
        br = self.n_back_right.get_abs()
        return [fl, fr, bl, br]
