"""
A simple turtle graphics simulation to test controller inputs without a physical rover.
"""
import turtle

from .constants import TwistOptions
from .controllers.base_station_state import BaseStationState
from .controllers.desktop_controller import DesktopController
from .controllers.physical_controller import PhysicalController


class TurtleRover:
    instance = None

    def __init__(self):
        """ Opens a turtle window for testing inputs. """
        self.turtle = turtle.Turtle()
        self.screen = turtle.Screen()
        self.turtle.speed(1)

        self.screen.title("Turtle Rover Simulation")
        self.screen.tracer(0)
        TurtleRover.instance = self

        self.thrust = 0
        self.turning_thrust = 0

        # Set up update loop
        def check_commands():
            self.process_state()
            self.screen.ontimer(check_commands, 16)  # ~60 FPS
        check_commands()
        try:
            self.screen.mainloop()
        except turtle.Terminator:
            pass

    def process_state(self):
        """ Given the stored state, applies movement to the turtle. """
        speed = 5
        turning_speed = 5
        self.turtle.forward(self.thrust * speed)
        self.turtle.right(self.turning_thrust * turning_speed)
        self.screen.update()

def mock_publish(topic_name: str, twist_option, throttle: float):
    """
    A mock publish method that can be used to move the turtle, based on the controllers.
    :param topic_name: the name of the topic to publish to.
    :param twist_option: the twist option to modify.
    :param throttle: the throttle to set the twist option.
    """
    print(topic_name, twist_option, throttle)
    rover = TurtleRover.instance
    if rover is None:
        return
    if twist_option == TwistOptions.LINEAR_X:
        rover.thrust = throttle
    elif twist_option == TwistOptions.ANGULAR_X:
        rover.turning_thrust = throttle

def get_index(args, index, default):
    try:
        return args[index]
    except IndexError:
        return default

if __name__ == "__main__":
    state = BaseStationState()
    DesktopController(mock_publish, state)
    physical_controller = PhysicalController(mock_publish, state)
    TurtleRover()
    physical_controller.stop()
