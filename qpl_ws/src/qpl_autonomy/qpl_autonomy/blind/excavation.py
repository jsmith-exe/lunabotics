import time
from geometry_msgs.msg import Twist


class ExcavationSequence:
    """Simple time-based excavation sequence."""

    DRIVE_SPEED = 0.2
    DRIVE_DURATION_S = 5.0

    def __init__(self, cmd_vel_pub):
        self.cmd_vel_pub = cmd_vel_pub

        self.running = False
        self.start_time = None
        self.complete = False

    def start(self): # Start driving forward
        self.running = True
        self.complete = False
        self.start_time = time.monotonic()

    def update(self): # Run one update of the sequence
        if not self.running:
            return

        elapsed = time.monotonic() - self.start_time

        if elapsed < self.DRIVE_DURATION_S:
            command = Twist()
            command.linear.x = self.DRIVE_SPEED
            self.cmd_vel_pub.publish(command)
        else:
            self.stop()
            self.complete = True

    def stop(self): # Stop the rover
        command = Twist()
        command.linear.x = 0.0
        self.cmd_vel_pub.publish(command)
        self.running = False
        self.complete = False

    def is_complete(self): # Return True when the sequence has finished
        return self.complete