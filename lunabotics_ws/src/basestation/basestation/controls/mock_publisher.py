import turtle

from lunabotics_ws.src.basestation.basestation.controls.constants import Commands
from lunabotics_ws.src.basestation.basestation.controls.controllers.base_station_state import BaseStationState
from lunabotics_ws.src.basestation.basestation.controls.controllers.desktop_controller import DesktopController
from lunabotics_ws.src.basestation.basestation.controls.controllers.physical_controller import PhysicalControllerHandler


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

def mock_publish(command: str, *args):
    """
    A mock publish method that can be used to move the turtle, based on the controllers.
    :param command: an initial command.
    :param args: arbitrary list of arguments.
    """
    print(args)
    rover = TurtleRover.instance
    if rover is None:
        return

    default = 1

    thrust = None
    turning_thrust = None
    match command:
        case Commands.FORWARD:
            thrust = get_index(args, 0, default)
        case Commands.REVERSE:
            thrust = -get_index(args, 0, default)
        case Commands.LEFT:
            turning_thrust = -get_index(args, 0, default)
        case Commands.RIGHT:
            turning_thrust = get_index(args, 0, default)
        case Commands.STOP_SIGNAL:
            command = args[0]
            if command == Commands.FORWARD or command == Commands.REVERSE: thrust = 0
            else: turning_thrust = 0

    if thrust is not None: rover.thrust = thrust
    if turning_thrust is not None: rover.turning_thrust = turning_thrust

def get_index(args, index, default):
    try:
        return args[index]
    except IndexError:
        return default

if __name__ == "__main__":
    state = BaseStationState()
    DesktopController(mock_publish, state)
    physical_controller = PhysicalControllerHandler(mock_publish, state)
    TurtleRover()
    physical_controller.stop()
