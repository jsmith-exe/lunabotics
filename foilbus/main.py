"""
FoilBus is a SPARK MAX simulator - a stand in for the motor and actuator setup.
Advantages:
 - No wiring
 - No access to physical rover needed

With socat installed, run:
socat PTY,link=/dev/ttyUSB0,rawer PTY,link=/tmp/fake_can_rx,rawer
This creates a fake USB /dev/ttyUSB0, which talks to /tmp/fake_can_rx, which will be written to by this script.
The diffdrive canbus system will connect to the fake USB.
"""
import argparse, os, struct, time, serial
from sparkmax_definitions import *

# Device IDs present on this simulated bus, matching the robot's wiring.
ALL_IDS      = [1, 2, 3, 4, 5, 6, 7]   # 1-4: wheels, 5-6: actuators, 7: drum
ACTUATOR_IDS = {5, 6}                    # these also send analog voltage (Status 3)


def main():
    ap = argparse.ArgumentParser(description="SPARK MAX simulator for diffdrive_canbus")
    ap.add_argument("--port", default="/tmp/fake_can_rx", help="Serial port (socat PTY end)")
    ap.add_argument("--baud", type=int, default=2000000, help="Must match serial_baud_rate in ros2_control params")
    args = ap.parse_args()

    # Wait for socat to create the PTY symlink before trying to open it.
    # socat may take a moment after launch, so poll rather than failing immediately.
    print(f"Waiting for {args.port} ...")
    while not os.path.exists(args.port):
        time.sleep(0.1)

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.005)
    print(f"Opened {args.port} — simulating IDs {ALL_IDS}")

    # Motor state per device: [commanded_rpm, measured_rpm, position_rotations]
    # Wheels and drum use RPM; actuators use duty cycle separately.
    state = {i: [0.0, 0.0, 0.0] for i in ALL_IDS}

    # Actuator state per device: [commanded_duty (-1 to 1), position (0 to 1)]
    # Position 0.0 = fully retracted, 1.0 = fully extended.
    act = {i: [0.0, 0.5] for i in ACTUATOR_IDS}

    # Pre-compute expected CAN IDs for each device's setpoint commands.
    # The plugin uses packed api_ids: 0x02 for duty cycle, 0x12 for velocity.
    DUTY_IDS = {create_packed_id(0x02, i): i for i in ALL_IDS}
    VEL_IDS  = {create_packed_id(0x12, i): i for i in ALL_IDS}

    next_tx   = time.monotonic()
    next_stat = time.monotonic()
    TX_PERIOD = 1.0 / 50    # 50 Hz — fast enough to keep plugin feedback stable

    while True:
        # --- RX: decode one incoming frame per loop iteration ---
        # The serial timeout (0.005s) means this blocks for at most 5ms
        # when no data is available, which limits the loop rate under low load.
        frame = read_waveshare_frame_from_serial(ser)
        if frame:
            can_id, data = frame
            if can_id == HEARTBEAT_ID:
                pass  # absorb heartbeat — no response needed
            elif can_id in DUTY_IDS and len(data) >= 4:
                dev  = DUTY_IDS[can_id]
                duty = struct.unpack_from("<f", data)[0]   # float32 LE
                if dev in ACTUATOR_IDS:
                    act[dev][0] = max(-1.0, min(1.0, duty))
                else:
                    # Approximate RPM from duty: NEO free-speed ~5700 RPM at full duty
                    state[dev][0] = duty * 5700.0
            elif can_id in VEL_IDS and len(data) >= 4:
                dev = VEL_IDS[can_id]
                state[dev][0] = struct.unpack_from("<f", data)[0]   # RPM as float32 LE

        # --- Dynamics: first-order lag toward commanded value ---
        # dt is capped at 0.1s to prevent large jumps after pauses.
        now = time.monotonic()
        dt  = min(now - next_tx + TX_PERIOD, 0.1)

        for dev in ALL_IDS:
            cmd, meas, pos = state[dev]
            # Alpha: how far to step toward target this tick.
            # Derived from e^(-dt/tau) where tau=0.15s is the time constant.
            # At tau, the response reaches ~63% of the commanded value.
            alpha = 1.0 - 2.718 ** (-dt / 0.15)
            meas += alpha * (cmd - meas)
            pos  += meas / 60.0 * dt   # RPM -> rotations/sec -> rotations
            state[dev] = [cmd, meas, pos]

        for dev in ACTUATOR_IDS:
            duty, pos = act[dev]
            # Full travel (0->1) takes 2 seconds at full duty (speed=0.5 pos/sec)
            pos = max(0.0, min(1.0, pos + duty * 0.5 * dt))
            act[dev] = [duty, pos]

        # --- TX: broadcast status frames at 50 Hz ---
        if now >= next_tx:
            next_tx = now + TX_PERIOD
            for dev in ALL_IDS:
                cmd, meas, pos = state[dev]
                applied = max(-1.0, min(1.0, meas / 5700.0))
                ser.write(create_applied_output_status(dev, applied))
                ser.write(create_velocity_status(dev, meas))
                ser.write(create_position_status(dev, pos))
                if dev in ACTUATOR_IDS:
                    # Map actuator position (0-1) back to voltage range
                    voltage = 0.279 + act[dev][1] * (1.85 - 0.279)
                    ser.write(create_analog_status(dev, voltage))

        # --- Stats: print motor state every 5 seconds ---
        if now >= next_stat:
            next_stat = now + 5.0
            info = "  ".join(
                f"id={d} rpm={state[d][1]:+.0f}" +
                (f" apos={act[d][1]:.2f}" if d in ACTUATOR_IDS else "")
                for d in ALL_IDS
            )
            print(f"[{now:.1f}] {info}")

if __name__ == "__main__":
    main()
