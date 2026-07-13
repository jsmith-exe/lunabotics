import argparse
from .core.devices import Motor, Actuator
from .core.can_sim import CANSim
from .ui import SimWindow

def main():
    ap = argparse.ArgumentParser(description="SPARK MAX simulator for diffdrive_canbus")
    ap.add_argument("--port", default="/tmp/fake_can_rx", help="Serial port (socat PTY end)")
    ap.add_argument("--baud", type=int, default=2000000, help="Must match serial_baud_rate in ros2_control params")
    args = ap.parse_known_args()[0]
    print(f"Args: {args}")

    can_setup = {
        1: Motor(1),
        2: Motor(2),
        3: Motor(3),
        4: Motor(4),
        5: Actuator(5),
        6: Actuator(6),
        7: Motor(7),
    }

    sim = CANSim(can_setup, args.port, args.baud)
    sim.start()
    ui = SimWindow(can_setup)
    ui.start()

if __name__ == "__main__":
    main()
