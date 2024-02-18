import wpilib
import commands2.button

from toolkit.oi import (
    XBoxController,
    LogitechController,
    JoystickAxis,
    Joysticks,
    DefaultButton,
)

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController


class Controllers:
    DRIVER: int = 0
    OPERATOR: int = 1

    DRIVER_CONTROLLER = wpilib.Joystick(0)
    OPERATOR_CONTROLLER = wpilib.Joystick(1)


class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[1])
        DRIVE_ROTATION_AXIS = JoystickAxis(
            Controllers.DRIVER, controllerDRIVER.R_JOY[0]
        )
        RESET_GYRO = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.B
        )
        X_MODE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X
        )
        
    class Elevator:
        ELEVATOR_HIGH = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.Y
        )
        ELEVATOR_MID = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.B
        )
        ELEVATOR_LOW = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.A
        )
        CLIMB = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.START
        )
        
        AMP = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.X
        )
    class Intake:
        # INTAKE_IN = commands2.button.Trigger(
        #     lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.RT) > 0.5
        # )
        
        INTAKE_IN = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > 0.5
        )
        
        INTAKE_OUT = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.LT) > 0.5
        )
    class Shooter:
        ...
        # AIM = commands2.button.Trigger(
        #     lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > 0.5
        # )
        
        AIM = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.RT) > 0.5
        )
        
    class Feeder:
        FEED = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.RB
        )
        UNFEED = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.LB
        )
        
        CLEAR_NOTE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X
        )
        
        # UNJAM =
         
