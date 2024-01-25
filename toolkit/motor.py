from units.SI import radians_per_second, radians


class Motor:
    def init(self): ...

    def set_raw_output(self, x: float): ...


class EncoderMotor(Motor):
    def get_sensor_position(self) -> radians: ...

    def get_sensor_velocity(self) -> radians_per_second: ...

    def set_sensor_position(self, pos: radians): ...


class PIDMotor(EncoderMotor):
    def set_target_position(self, pos: radians): ...

    def set_target_velocity(self, vel: radians_per_second): ...
