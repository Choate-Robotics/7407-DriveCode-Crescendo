from dataclasses import dataclass

import commands2 as commands
import commands2.button

from toolkit.oi.joysticks import Joysticks


@dataclass
class Button:
    controller_id: int

    def __call__(self) -> commands.Trigger: ...


@dataclass
class DefaultButton(Button):
    """
    Wrapper for wpilib button
    """
    button_id: int

    def __call__(self) -> commands.Trigger:
        if self.button_id < 0:
            return commands.Trigger(
                lambda: Joysticks.joysticks[self.controller_id].getRawAxis(-self.button_id) > 0.8
            )
        return commands.Trigger(
            lambda: Joysticks.joysticks[self.controller_id].getRawButton(self.button_id)
        )


@dataclass
class AxisButton(Button):
    """
    Wrapper for wpilib axis button
    """
    axis_id: int
    range_min: float = -1
    range_max: float = 1

    def __call__(self) -> commands.Trigger:
        return commands.Trigger(
            lambda: self.range_min <= Joysticks.joysticks[self.controller_id].getRawAxis(self.axis_id) <= self.range_max
        )
