
#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

from dataclasses import Field
import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
from wpilib import Field2d, SmartDashboard

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.robot = robot
        self.field = Field2d()
        # Uhh.. the robot should probably update this, NOT the simulation
        SmartDashboard.putData('Field', self.field)
        print("TODO: modify simulation for my robot")

        # Change these parameters to fit your robot!

        # By using the units we can mix metric and imperial units
        # if that is your thing. Some parts will be dictated in imperial
        # units so best to stick with them.
        bumper_width = 3.25 * units.inch

        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_FALCON_500,  # motor configuration
            90 * units.lb,  # robot mass
            2.0,  # drivetrain gear ratio
            1,  # motors per side
            55*units.cm,  # robot wheelbase
            52*units.cm + bumper_width*2,  # robot width
            96*units.cm + bumper_width*2,  # robot length
            6*units.inch,  # wheel diameter
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.robot.left_power.output
        r_motor = self.robot.right_power.output
        transform = self.drivetrain.calculate(l_motor, -r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)
        # Update the robot's field position
        # TODO: Remove this and instead update the encoders
        # properly; then let the robot figure out where it went.
        self.field.setRobotPose(pose)


