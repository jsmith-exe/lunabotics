import turtle

from lunabotics.basestation.controls.constants import Commands
from lunabotics.basestation.controls.desktop_controller import DesktopController


class TurtleRover:
    instance = None

    def __init__(self):
        self.turtle = turtle.Turtle()
        self.screen = turtle.Screen()
        self.turtle.speed(1)

        self.screen.title("Turtle Rover Simulation")
        self.screen.tracer(0)
        TurtleRover.instance = self

        self.command_state = dict()
        for command in Commands:
            self.command_state[command] = False

        def check_commands():
            self.process_state()
            self.screen.ontimer(check_commands, 16)  # ~60 FPS
        check_commands()
        try:
            self.screen.mainloop()
        except turtle.Terminator:
            pass

    def process_state(self):
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
        self.command_state[command] = state

def mock_publish(command: str, *args):
    rover = TurtleRover.instance
    if rover is None:
        return
    if command == Commands.STOP_SIGNAL:
        command_to_stop = args[0]
        rover.update_command_state(command_to_stop, False)
    else:
        rover.update_command_state(command, True)

if __name__ == "__main__":
    DesktopController(mock_publish)
    TurtleRover()
