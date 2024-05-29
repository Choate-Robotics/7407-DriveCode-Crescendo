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
        RESET_GYRO = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getPOV() == 180
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
        
        
        AMP = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.X
        )
    class Intake:
        INTAKE_IN = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.RT) > 0.4
        )
        
        # INTAKE_IN = commands2.button.Trigger(
        #     lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.4
        # )
        
        
        INTAKE_OUT = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.LT) > 0.4
        )
        
        AUTO_INTAKE = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.4
        )
        
        
    class Shooter:
        ...
        AIM = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > 0.4
        )

        # AIM_DRIVETRAIN_RIGHT = commands2.button.Trigger(
        #     lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.4
        # )
        
        STATIC_FEED_SHOT = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.LB
        )
        
        SET_WRIST_SUBWOOFER = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 90
        )
        
        ENABLE_AIM_WRIST_OPERATOR = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 270
        )
        
        FLYWHEEL_MANUAL_REVERSE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.R_3
        )
        
        AMP = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.RB
        )
        
        FEED_SHOT = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.B
        )
        
        FEED_MIDLINE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.START
        )
        
        
        
    class Feeder:
        FEED = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.RB
        )
        UNFEED = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.LB
        )
        
        DUMP_NOTE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.A
        )
        
        # CLEAR_NOTE =
        
        FEED_NOTE_FLYWHEEL = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.L_JOY[1]) > 0.4
        )
        
        # UNJAM =
         
    class Climb:
        
        CLIMB_UP = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.SELECT
        )
        
        CLIMB_DOWN = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.START
        )
        
        TRAP =  commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 0
        )
        
        UNDO_CLIMB_UP = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 180
        )
        
        # LOCK_STAGE_SOURCE = commands2.button.JoystickButton(
        #     Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X
        # )
        
        # LOCK_STAGE_AMP = commands2.button.JoystickButton(
        #     Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.Y
        # )
        
