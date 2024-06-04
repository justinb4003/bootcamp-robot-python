from wpilib import Joystick, SmartDashboard
from commands2 import ( ParallelCommandGroup, TimedCommandRobot, SequentialCommandGroup,
                        InstantCommand, CommandScheduler,
)
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut, VelocityDutyCycle


# We can make an alias for some commonly used functions like this
sdpn = SmartDashboard.putNumber
sdpb = SmartDashboard.putBoolean



class DriverController():

    def __init__(self, joystick: Joystick):
        self.joystick = joystick

    def get_drive_left(self) -> float:
        return -self.joystick.getRawAxis(1)

    def get_drive_right(self) -> float:
        return -self.joystick.getRawAxis(3)


class MyRobot(TimedCommandRobot):
    left_drive = TalonFX(10)
    right_drive = TalonFX(20)
    left_power = DutyCycleOut(0)
    right_power = DutyCycleOut(0)
    driver_joystick = Joystick(0)
    driver_controller = DriverController(driver_joystick)

    def robotInit(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        left_speed = self.driver_controller.get_drive_left()
        right_speed = self.driver_controller.get_drive_right()
        sdpn('drivetrain/left_speed', left_speed)
        sdpn('drivetrain/right_speed', right_speed)
        self.left_power.output = left_speed
        self.right_power.output = right_speed
        self.left_drive.set_control(self.left_power)
        self.right_drive.set_control(self.right_power)

        left_encoder = self.left_drive.get_position().value
        sdpn('drivetrain/encoder/left', left_encoder)
        pass
