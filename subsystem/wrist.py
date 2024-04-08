import math
from math import pi

import config
import ntcore
import constants
from toolkit.motors.rev_motors import SparkMax
from toolkit.subsystem import Subsystem
from toolkit.utils.toolkit_math import bounded_angle_diff
from units.SI import radians
from wpilib import DigitalInput

class Wrist(Subsystem):
    def __init__(self):
        super().__init__()
        self.wrist_motor = SparkMax(
            can_id=config.wrist_motor_id, inverted=True, config=config.WRIST_CONFIG,
            config_others=[config.WRIST_AIM_CONFIG]
        )

        self.feed_motor = SparkMax(
            can_id=config.feed_motor_id, inverted=True, config=config.FEED_CONFIG
        )
        self.note_staged: bool = False
        self.wrist_zeroed: bool = False
        self.rotation_disabled: bool = False
        self.feed_disabled: bool = False
        self.beam_break_first: DigitalInput = None
        self.beam_break_second: DigitalInput = None   
        self.disable_rotation: bool = False
        self.locked: bool = False
        self.ready_to_shoot: bool = False
        self.target_angle: radians = 0
        self.wrist_moving: bool = False

    def init(self):
        self.wrist_motor.init()
        self.table = ntcore.NetworkTableInstance.getDefault().getTable('wrist')
        self.wrist_motor.optimize_sparkmax_absolute_encoder(30)
        # self.wrist_motor.pid_controller.setIMaxAccum(.00)
        self.wrist_abs_encoder = self.wrist_motor.abs_encoder()
        self.feed_motor.init()
        self.feed_motor.optimize_sparkmax_no_position()
        self.beam_break_first = DigitalInput(config.feeder_beam_break_first_channel)
        self.beam_break_second = DigitalInput(config.feeder_beam_break_second_channel)
        self.table.getSubTable('wrist motor').putNumber('P', config.WRIST_AIM_CONFIG.k_P)
        self.table.getSubTable('wrist motor').putNumber('I', config.WRIST_AIM_CONFIG.k_I)
        self.table.getSubTable('wrist motor').putNumber('D', config.WRIST_AIM_CONFIG.k_D)
        
        
        
    def update_wrist_pid(self):
        
        WP = self.table.getSubTable('wrist motor').getNumber('P', config.WRIST_AIM_CONFIG.k_P)
        WI = self.table.getSubTable('wrist motor').getNumber('I', config.WRIST_AIM_CONFIG.k_I)
        WD = self.table.getSubTable('wrist motor').getNumber('D', config.WRIST_AIM_CONFIG.k_D)
        
        self.wrist_motor.pid_controller.setP(
            WP,
            1
        )
        
        self.wrist_motor.pid_controller.setI(
            WI,
            1
        )
        
        self.wrist_motor.pid_controller.setD(
            WD,
            1
        )

    def limit_angle(self, angle: radians) -> radians:
        if self.locked and angle <= constants.wrist_min_rotation_stage:
            return constants.wrist_min_rotation_stage
        if angle <= constants.wrist_min_rotation:
            return constants.wrist_min_rotation
        elif angle >= constants.wrist_max_rotation:
            return constants.wrist_max_rotation
        return angle
    
    def check_value_type(self, value: float) -> bool:
        return isinstance(value, float) or isinstance(value, int)
    
    @staticmethod
    def abs_to_radians(abs_angle: float) -> radians:
        if abs_angle > .5:
            return (1 - abs_angle) * -2 * math.pi
        else:
            return abs_angle * 2 * math.pi
        
    @staticmethod
    def radians_to_abs(angle: radians) -> float:
        if angle < 0:
            return 1 - (angle / (-2 * math.pi))
        else:
            return angle / (2 * math.pi)

    # wrist methods
    def set_wrist_angle(self, angle: radians, slot=0):
        """
        Sets the wrist angle to the given position
        :param pos: The position to set the wrist to(float)
        :return: None
        """
        
        if not self.check_value_type(angle):
            if config.DEBUG_MODE:
                raise ValueError("Angle must be a float or int")
            return
        
        angle = self.limit_angle(angle)
        self.target_angle = angle

        current_angle = self.get_wrist_angle()

        ff = config.wrist_max_ff * math.cos(angle - config.wrist_ff_offset)

        if not self.rotation_disabled:
            self.wrist_motor.set_target_position(
                (angle / (pi * 2)) * constants.wrist_gear_ratio,
                ff,# if angle < current_angle else 0,
                slot=slot
            )
            
    def aim_wrist(self, angle: radians):
        """
        Sets the wrist angle to the given position
        :param pos: The position to set the wrist to(float)
        :return: None
        """
        
        if not self.check_value_type(angle):
            if config.DEBUG_MODE:
                raise ValueError("Angle must be a float or int")
            return
        
        angle = self.limit_angle(angle)
        self.target_angle = angle
        
        ff = config.wrist_max_ff * math.cos(angle - config.wrist_ff_offset)

        if not self.rotation_disabled:
            self.wrist_motor.set_target_position(
                (angle / (pi * 2)) * constants.wrist_gear_ratio,
                ff - .2,# if angle < current_angle else 0,
                slot=1
            )
            

    def get_wrist_angle(self):
        """
        Gets the wrist rotation in radians
        :return:
        """
        return (
                (self.wrist_motor.get_sensor_position() / constants.wrist_gear_ratio)
                * pi
                * 2
        )
        
        # return self.get_wrist_abs_angle()

    def note_detected(self) -> bool:
        return not self.beam_break_second.get()

    def detect_note_first(self) -> bool:
        return not self.beam_break_first.get()
    
    def detect_note_second(self) -> bool:
        return not self.beam_break_second.get()
    
    def note_in_feeder(self) -> bool:
        return self.detect_note_first() or self.detect_note_second()

    def is_at_angle(self, angle: radians, threshold=math.radians(2)):
        """
        Checks if the wrist is at the given angle
        :param angle: The angle to check for
        :param threshold: The threshold to check for
        :return: True if the wrist is at the given angle, False otherwise
        """
        if not math.isfinite(self.get_wrist_angle()):
            return False # usually means the encoder is not connected/simulation
        
        return abs(bounded_angle_diff(self.get_wrist_angle(), angle)) < threshold
    
    def get_wrist_velocity(self):
        return self.wrist_motor.get_sensor_velocity() * (2 * math.pi) / constants.wrist_gear_ratio

    def get_wrist_abs_angle(self):

        angle = self.wrist_abs_encoder.getPosition() - config.wrist_zeroed_pos

        return self.abs_to_radians(angle)

    def zero_wrist(self) -> None:  # taken from cyrus' code
        # Reset the encoder to zero

        pos = (self.get_wrist_abs_angle() / (2 * math.pi)) * constants.wrist_gear_ratio
        print(math.degrees(self.get_wrist_abs_angle()))
        print(pos)
        self.wrist_motor.set_sensor_position(
            pos
        )
        self.wrist_zeroed = True

    # feed in methods
    def feed_in(self):
        if not self.feed_disabled:
            # self.feed_motor.set_target_velocity(config.feeder_velocity)
            # self.feed_motor.set_raw_output(config.feeder_velocity)
            self.feed_motor.set_target_voltage(config.feeder_voltage_feed)

    def set_feed_voltage(self, voltage: float):
        self.feed_motor.set_target_voltage(voltage)

    def feed_out(self):
        if not self.feed_disabled:
            # self.feed_motor.set_target_velocity(-(config.feeder_velocity))
            # self.feed_motor.set_raw_output(-(config.feeder_velocity))
            self.feed_motor.set_target_voltage(-config.feeder_voltage_trap)

    def stop_feed(self):
        # self.feed_motor.set_target_position(self.feed_motor.get_sensor_position())
        # self.feed_motor.set_raw_output(0)
        self.feed_motor.set_target_voltage(0)
        
    def feed_idle(self):
        self.feed_motor.set_target_voltage(3)

    def feed_note(self):
        if not self.feed_disabled:
            # self.feed_motor.set_raw_output(config.feeder_pass_velocity)
            self.feed_motor.set_target_voltage(config.feeder_pass_voltage)

    def set_note_staged(self):
        self.note_staged = True

    def set_note_not_staged(self):
        self.note_staged = False

    def lock(self):
        self.locked = True

    def unlock(self):
        self.locked = False

    def periodic(self) -> None:
        # self.zero_wrist()
        

        # table.putNumber('wrist angle', math.degrees(self.get_wrist_angle()))
        self.table.putNumber('wrist abs angle', math.degrees(self.get_wrist_abs_angle()))
        # table.putNumber('wrist abs raw', self.wrist_abs_encoder.getPosition())
        self.table.putNumber('wrist angle', math.degrees(self.get_wrist_angle()))
        self.table.putBoolean('note in feeder', self.note_in_feeder())
        self.table.putBoolean('note detected', self.note_detected())
        self.table.putBoolean('wrist zeroed', self.wrist_zeroed)
        self.table.putBoolean('ready to shoot', self.ready_to_shoot)
        self.table.putBoolean('first beam break', not self.beam_break_first.get())
        self.table.putBoolean('second beam break', not self.beam_break_second.get())
        self.table.putBoolean('rotation disabled', self.rotation_disabled)
        self.table.putBoolean('feed disabled', self.feed_disabled)
        self.table.putBoolean('locked', self.locked)
        self.table.putNumber('target angle', math.degrees(self.target_angle))
        self.table.putNumber('target angle raw', self.radians_to_abs(self.target_angle))
        self.table.putBoolean('wrist moving', self.wrist_moving)
        self.table.putNumber('wrist current', self.wrist_motor.motor.getOutputCurrent())
        self.table.putNumber('wrist applied output', self.wrist_motor.motor.getAppliedOutput())
