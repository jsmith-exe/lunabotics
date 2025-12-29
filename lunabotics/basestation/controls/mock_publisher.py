import turtle

from lunabotics.basestation.controls.constants import Commands
from lunabotics.basestation.controls.controllers.base_station_state import BaseStationState
from lunabotics.basestation.controls.controllers.desktop_controller import DesktopController
from lunabotics.basestation.controls.controllers.physical_controller import PhysicalControllerHandler


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

        self.command_state = dict()
        for command in Commands:
            self.command_state[command] = False

        # Set up update look
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
        if self.command_state[Commands.FORWARD]:
            self.turtle.forward(speed)
        if self.command_state[Commands.LEFT]:
            self.turtle.left(speed)
        if self.command_state[Commands.RIGHT]:
            self.turtle.right(speed)
        if self.command_state[Commands.REVERSE]:
            self.turtle.backward(speed)

        self.screen.update()

    def update_command_state(self, command: str, state: bool):
        """ Updates some state of the command. """
        self.command_state[command] = state

def mock_publish(command: str, *args):
    """
    A mock publish method that can be used to move the turtle, based on the controllers.
    :param command: an initial command.
    :param args: arbitrary list of arguments.
    """
    rover = TurtleRover.instance
    if rover is None:
        return

    if command == Commands.STOP_SIGNAL:
        command_to_stop = args[0]
        rover.update_command_state(command_to_stop, False)
        return

    rover.update_command_state(command, True)

if __name__ == "__main__":
    state = BaseStationState()
    DesktopController(mock_publish, state)
    physical_controller = PhysicalControllerHandler(mock_publish, state)
    TurtleRover()
    physical_controller.stop()
