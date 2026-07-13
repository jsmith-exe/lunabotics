# SPARK MAX simulator
A virtual replacement for the motor and actuator CAN bus setup.
Uses a virtual USB port and a Python script to recreate the motor responses.

**Run with** `qpl_can_sim` - this will create the fake USB with `socat` and start the Python script to simulate the CAN bus.

Advantages:
- No wiring
- No access to physical rover needed

Disadvantages:
- Likely inaccurate


The socat command is this:
```bash
socat PTY,link=/dev/ttyUSB0,rawer PTY,link=/tmp/fake_can_rx,rawer
```
This creates a fake USB `/dev/ttyUSB0`, which talks to `/tmp/fake_can_rx`, which will be written to by this script.
The diffdrive canbus system will connect to the fake USB.
