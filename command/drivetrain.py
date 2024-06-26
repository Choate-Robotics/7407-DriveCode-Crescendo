from __future__ import annotations
import logging
import math
import time

import wpilib

from robot_systems import Field
from toolkit.command import SubsystemCommand

import config, constants
from subsystem import Drivetrain
from sensors import TrajectoryCalculator, Limelight
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from toolkit.utils.toolkit_math import bounded_angle_diff
from math import radians
from wpimath.units import seconds
import robot_states as states
import ntcore
from enum import Enum
from wpimath.geometry import Rotation2d

from wpilib import RobotState
from utils import POI



def curve(x):
    if abs(x) < 0.11:
        return 0
    if x < 0:
        return 1.12*(x + .11)
    return 1.12*(x - .11)

def bound_angle(degrees: float):
    degrees = degrees % 360
    if degrees > 180:
        degrees -= 360
    return degrees


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    """
    Main drive command
    """
    driver_centric = False
    driver_centric_reversed = True

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * (1 if config.drivetrain_reversed else -1),
            self.subsystem.axis_dy.value * (1 if config.drivetrain_reversed else -1),
            self.subsystem.axis_rotation.value,
        )

        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)

        # dx *= self.subsystem.max_vel
        dx *= states.drivetrain_controlled_vel
        # dy *= -self.subsystem.max_vel
        dy *= states.drivetrain_controlled_vel
        
        # d_theta *= self.subsystem.max_angular_vel
        d_theta *= states.drivetrain_controlled_angular_vel
        

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, dx), -d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DriveSwerveAim(SubsystemCommand[Drivetrain]):
    """
    Aim drivetrain at speaker based on shooter calculations
    """
    driver_centric = False
    driver_centric_reversed = True
    
    class Target(Enum):
        speaker = 0
        feed = 1
        static_feed = 2
        feed_amp = 3
        amp = 4
        stage_amp = 5
        stage_source = 6

    def __init__(self, drivetrain, target_calc: TrajectoryCalculator, target: Target = Target.speaker, limit_speed: bool = True, auto:bool = False):
        super().__init__(drivetrain)
        self.target_calc = target_calc
        self.target = target
        self.limit_speed = limit_speed
        # self.theta_controller = PIDController(0.0075, 0, 0.0001, config.period)
        self.theta_controller = PIDController(
            config.drivetrain_rotation_P, config.drivetrain_rotation_I, config.drivetrain_rotation_D,
            config.period
            )
        self.shooting: bool = False
        # self.theta_controller.setTolerance(radians(3 if RobotState.isAutonomous() else 3), radians(4 if RobotState.isAutonomous() else 4))
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('Drivetrain Aim')
        self.auto = auto

    def initialize(self) -> None:
        self.shooting = True
        self.theta_controller.enableContinuousInput(radians(-180), radians(180))
        self.theta_controller.reset()
        if config.drivetrain_rotation_enable_tuner:
            self.table.putNumber('P', config.drivetrain_rotation_P)
            self.table.putNumber('I', config.drivetrain_rotation_I)
            self.table.putNumber('D', config.drivetrain_rotation_D)
            self.table.putNumber('tolerance', 2)
            self.table.putNumber('velocity tolerance', 1)
            self.table.putNumber('drivetrain offset', config.drivetrain_aiming_offset)


    def execute(self) -> None:
        
        if (
            self.target == DriveSwerveAim.Target.speaker
        ):
            self.shooting = True
            self.target_calc.odometry.enable_shooting()
        else:
            self.shooting = False
            self.target_calc.odometry.disable_shooting()
        
        if config.drivetrain_rotation_enable_tuner:
            config.drivetrain_rotation_P = self.table.getNumber('P', config.drivetrain_rotation_P)
            config.drivetrain_rotation_I = self.table.getNumber('I', config.drivetrain_rotation_I)
            config.drivetrain_rotation_D = self.table.getNumber('D', config.drivetrain_rotation_D)
            self.theta_controller.setP(config.drivetrain_rotation_P)
            self.theta_controller.setI(config.drivetrain_rotation_I)
            self.theta_controller.setD(config.drivetrain_rotation_D)
            # self.theta_controller.setTolerance(radians(self.table.getNumber('tolerance', 2)), radians(self.table.getNumber('velocity tolerance', 1)))
            

            
            config.drivetrain_aiming_offset = self.table.getNumber('drivetrain offset', config.drivetrain_aiming_offset)
            # put graphs
        
        self.theta_controller.setTolerance(
            self.target_calc.get_shot_pos_tolerance() * (2 if self.auto else 1)
            if self.target == DriveSwerveAim.Target.speaker
            else config.drivetrain_feed_tolerance
        )
        
        dx, dy = (
            self.subsystem.axis_dx.value * (1 if config.drivetrain_reversed else -1),
            self.subsystem.axis_dy.value * (1 if config.drivetrain_reversed else -1),
        )

        
        
        def get_target_angle():
            match self.target:
                case DriveSwerveAim.Target.speaker:
                    return self.target_calc.get_bot_theta()
                case DriveSwerveAim.Target.feed:
                    return self.target_calc.get_bot_theta_feed()
                case DriveSwerveAim.Target.static_feed:
                    return self.target_calc.get_bot_theta_static_feed()
                case DriveSwerveAim.Target.feed_amp:
                    return self.target_calc.get_bot_theta_feed(True)
                case DriveSwerveAim.Target.amp:
                    return Rotation2d(radians(-90)) if config.active_team == config.Team.RED else Rotation2d(radians(90))
                case DriveSwerveAim.Target.stage_amp:
                    return POI.Coordinates.Structures.Stage.kLeft.get().rotation()
                case DriveSwerveAim.Target.stage_source:
                    return POI.Coordinates.Structures.Stage.kRight.get().rotation()
                case _:
                    return self.target_calc.get_bot_theta()

        target_angle = get_target_angle()
        
        
        # current = self.subsystem.odometry_estimator.getEstimatedPosition().rotation().radians()
        current = self.subsystem.get_heading().radians()
        current -= radians(config.drivetrain_aiming_offset)
        d_theta = self.theta_controller.calculate(current, target_angle.radians())
        if config.drivetrain_rotation_enable_tuner:
            self.table.putNumber('target angle', target_angle.radians())
            self.table.putNumber('current angle', current)
            self.table.putNumber('error', self.theta_controller.getPositionError())
            self.table.putNumber('velocity error', self.theta_controller.getVelocityError())
        
        def drive_speed():
            if not self.limit_speed:
                return 0
            vx = self.subsystem.chassis_speeds.vx
            vy = self.subsystem.chassis_speeds.vy
            v_total = math.sqrt(vx ** 2 + vy ** 2)
            return v_total
        
        def robot_angle():
            pitch = self.subsystem.gyro.get_robot_pitch()
            return abs(pitch)
        
        if self.theta_controller.atSetpoint() and drive_speed() < config.drivetrain_aiming_move_speed_threshold\
            and robot_angle() < config.drivetrain_aiming_tilt_threshold and self.shooting:
            self.subsystem.ready_to_shoot = True
        else:
            self.subsystem.ready_to_shoot = False
        dx = curve(dx)
        dy = curve(dy)

        dx *= states.drivetrain_controlled_vel
        dy *= states.drivetrain_controlled_vel
        # d_theta *= config.drivetrain_aiming_max_angular_speed

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, dx), d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.target_calc.odometry.disable_shooting()
        self.subsystem.ready_to_shoot = False
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DrivetrainZero(SubsystemCommand[Drivetrain]):
    """
    Zeroes drivetrain
    """

    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        print("ZEROING DRIVETRAIN")
        self.subsystem.reset_gyro(radians(180))
        self.subsystem.n_front_left.zero()
        self.subsystem.n_front_right.zero()
        self.subsystem.n_back_left.zero()
        self.subsystem.n_back_right.zero()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        logging.info("Successfully re-zeroed swerve pods.")
        ...


class DriveSwerveHoldRotation(SubsystemCommand[Drivetrain]):
    driver_centric = False
    driver_centric_reversed = True

    def __init__(self,
                 subsystem: Drivetrain,
                 theta_f: radians,
                 threshold: float = math.radians(5),
                 max_angular_vel: float | None = None,
                 period: float = 0.02,
                 ):
        super().__init__(subsystem)
        max_angular_vel = max_angular_vel or subsystem.max_angular_vel
        self.controller = PIDController(
            9, 0, 0.003
        )

        self.controller.setTolerance(threshold)
        self.controller.enableContinuousInput(-math.pi, math.pi)
        self.controller.setTolerance(radians(3))

        self.theta_f = theta_f

        self.start_time = 0
        self.t = 0

    def initialize(self) -> None:
        self.start_time = time.perf_counter()

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time

        current_theta = self.subsystem.odometry_estimator.getEstimatedPosition().rotation().radians()
        error = self.controller.calculate(current_theta, self.theta_f)
        d_theta = error

        dx, dy = (
            self.subsystem.axis_dx.value * (1 if config.drivetrain_reversed else -1),
            self.subsystem.axis_dy.value * (1 if config.drivetrain_reversed else -1),

        )

        dx = curve(dx)
        dy = curve(dy)

        dx *= states.drivetrain_controlled_vel
        dy *= states.drivetrain_controlled_vel
        # d_theta *= self.max_angular_vel

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, dx), d_theta)
        elif self.driver_centric_reversed:
            self.subsystem.set_driver_centric((-dy, dx), d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return self.t > 3 or self.controller.atSetpoint()

    def runsWhenDisabled(self) -> bool:
        return False


class DriveSwerveNoteLineup(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, LimeLight: Limelight):
        '''
        Lines up the robot with the target
        :param drivetrain: Drivetrain subsystem
        :param LimeLight: Limelight sensor
        '''
        super().__init__(subsystem)
        self.drivetrain = subsystem
        self.limelight = LimeLight
        self.target_exists = False
        # self.v_pid = PIDController(.09, 0, 0.1)
        self.h_pid = PIDController(.04, 0, 00)
        self.o_pid = PIDController(
            config.drivetrain_rotation_P,
            config.drivetrain_rotation_I,
            config.drivetrain_rotation_D
        )
        self.v_constant = 2.5
        self.is_pipeline: bool = False
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable('drivetrain pid tune')
        self.gyro_lock = 0
        
    def initialize(self):
        # self.limelight.set_pipeline_mode(config.LimelightPipeline.neural)
        # self.v_pid.reset()
        self.o_pid.enableContinuousInput(radians(-180), radians(180))
        self.o_pid.reset()
        self.h_pid.reset()
        self.gyro_lock = 0
        self.target_exists = False
        # self.v_pid.setTolerance(config.object_detection_ty_threshold)
        self.h_pid.setTolerance(config.object_detection_tx_threshold)
        
    def execute(self):
        self.limelight.update()
        self.is_pipeline = True
        # print('can see')
        if self.limelight.target_exists() == False or self.limelight.get_target() == None:
            self.target_exists = False
            # print('no target')
            # print(self.limelight.table.getNumber('tv', 3))
            self.drivetrain.set_robot_centric((0,0),0)
            return
        
        if self.limelight.target_exists() and self.target_exists == False:
            self.target_exists = True
            self.gyro_lock = self.drivetrain.get_heading().radians()
        else:
            self.target_exists = False
            return
        # self.nt.putBoolean('see target', True)
        # print("target")
        tx, ty, ta = self.limelight.get_target()
        
        self.nt.putNumber('tx target', config.object_detection_tx)
        self.nt.putNumber('ty target', config.object_detection_ty)
        
        self.nt.putNumber("tx", tx)
        self.nt.putNumber("ty", ty)
        # self.nt.putNumber('ta', ta)
        
        
        
        # if self.target_exists == False and self.target_exists:
        #     self.target_exists = True
            
        # print("Tracking...")
            
        # dy = self.v_pid.calculate(ty, config.object_detection_ty)
        dx = self.h_pid.calculate(tx, config.object_detection_tx)
        omega = self.o_pid.calculate(self.drivetrain.get_heading().radians(), self.gyro_lock)
        
        dy = self.v_constant ** (-(abs(self.h_pid.getPositionError()) / 30)) if abs(self.h_pid.getPositionError()) > 0 else 1
        
        
            
        # dx *= states.drivetrain_controlled_vel * config.object_detection_drivetrain_speed_dx
        dy *= states.drivetrain_controlled_vel * config.object_detection_drivetrain_speed_dy
            
        self.drivetrain.set_robot_centric((-dy, -dx), -omega)
        
            
    def isFinished(self):
        # return self.h_pid.atSetpoint() #and self.v_pid.atSetpoint()
        return False
    
    def end(self, interrupted: bool = False):
        # self.limelight.set_pipeline_mode(config.LimelightPipeline.feducial)
        self.drivetrain.set_robot_centric((0, 0), 0)


class DriveSwerveNoteRotate(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, limelight: Limelight):
        super().__init__(subsystem)
        self.drivetrain = subsystem
        self.limelight = limelight

        self.theta_pid = PIDController(0.3, 0, 0)
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt.getTable("drivetrain")

    def initialize(self):
        self.theta_pid.enableContinuousInput(radians(-180), radians(180))
        self.theta_pid.reset()
        self.theta_pid.setSetpoint(0)

    def execute(self):
        self.limelight.update()

        if not self.limelight.target_exists() or not self.limelight.get_target():
            self.drivetrain.set_robot_centric((0, 0), 0)
            return
        
        tx, ty, ta = self.limelight.get_target()

        omega = self.theta_pid.calculate(-tx)

        self.drivetrain.set_robot_centric((0, 0), omega)
        self.table.putNumber(-tx, "error")
        self.table.putNumber(0, "setpoint")
        self.table.putNumber(omega, "output")

    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted):
        self.drivetrain.set_robot_centric((0, 0), 0)

class DriveSwerveXMode(SubsystemCommand[Drivetrain]):
    def initialize(self):
        pass

    def execute(self):
        self.subsystem.x_mode()

    def isFinished(self):
        return False
    
    def end(self, interrupted):
        pass