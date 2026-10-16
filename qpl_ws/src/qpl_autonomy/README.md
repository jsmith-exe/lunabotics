# qpl_autonomy

ROS 2 package containing the mission-level autonomy logic for the QPL rover, including feedback-based excavation and deposition sequences, configuration, and launch files.

## Overview

`qpl_autonomy` provides the rover's high-level mechanical autonomy sequences.

The package currently contains two independent finite state machines (FSMs):

* Excavation
* Deposition

Each sequence is started through a command received from the Basestation. The sequences only run one after another under `AUTO`, which loops them with navigation in between.

The sequences themselves operate without localisation or navigation. They use actuator feedback and drivetrain odometry to determine when each step has completed. The optional `NAV_` commands use Nav2 to reach a waypoint first.

The package is structured so that physical tuning values can be changed through configuration without modifying the FSM implementation where possible.

## Package Structure

```text
qpl_autonomy/
├── config/
│   ├── fsm_tuning.yaml
│   └── legacy/
├── launch/
│   ├── autonomy.launch.py
│   └── legacy/
├── qpl_autonomy/
│   ├── __init__.py
│   ├── autonomy_node.py
│   ├── excavation.py
│   ├── deposition.py
│   ├── navigation.py
│   └── legacy/
└── ...
```

### `config/`

Configuration for autonomy update rates and FSM tuning values.

The active configuration is stored in:

```text
config/fsm_tuning.yaml
```

### `launch/`

Launch files for starting the autonomy node.

The active launch file is:

```text
launch/autonomy.launch.py
```

### `qpl_autonomy/`

Python nodes and FSM implementations belonging to the package.

The main ROS 2 node is:

```text
qpl_autonomy/autonomy_node.py
```

The individual autonomy sequences are implemented in:

```text
qpl_autonomy/excavation.py
qpl_autonomy/deposition.py
```

### `legacy/`

These files are kept for reference and are not part of the active autonomy system.

---

## Autonomy Node

The main node is:

```text
autonomy_node.py
```

The node receives high-level commands and manages the active autonomy sequence.

Commands are received through:

```text
/autonomy/command
```

using:

```text
std_msgs/msg/String
```

The currently supported commands are:

| Command        | Function                                                                  |
| -------------- | ------------------------------------------------------------------------- |
| `EXCAVATE`     | Start the excavation sequence where the rover is                          |
| `DEPOSIT`      | Start the deposition sequence where the rover is                          |
| `NAV_EXCAVATE` | Drive to the excavation waypoint with Nav2, then start excavation         |
| `NAV_DEPOSIT`  | Drive to the deposition waypoint with Nav2, then start deposition         |
| `AUTO`         | Full autonomy: repeat `NAV_EXCAVATE` then `NAV_DEPOSIT` until `STOP`      |
| `STOP`         | Stop the active sequence or navigation and return autonomy to `IDLE`      |

Shell aliases: `qpl_excavate`, `qpl_deposit`, `qpl_nav_excavate`, `qpl_nav_deposit`, `qpl_auto`, `qpl_stop`. The basestation has a button for each.

Only one sequence can be active at a time.

When autonomy is idle, an `EXCAVATE` or `DEPOSIT` command starts the corresponding sequence. A sequence must complete before autonomy returns to `IDLE`.

##### `STOP` does not start another sequence or attempt to reposition the rover. It stops autonomous motion and leaves the rover in its current state.

### Full autonomy (`AUTO` / `qpl_auto`)

```text
navigate to excavation waypoint -> excavate -> navigate to deposition waypoint -> deposit -> repeat
```

It loops until `STOP`. If navigation fails after its retries, `AUTO` ends and autonomy returns to `IDLE`.

### Navigation commands

`NAV_EXCAVATE`, `NAV_DEPOSIT` and `AUTO` need Nav2 running (`qpl_nav`) and the rover localised from the AprilTag. If Nav2 isn't running the command is refused; `EXCAVATE` and `DEPOSIT` never need it.

The waypoints are map-frame `[x, y, yaw]` values in `config/fsm_tuning.yaml` (`navigation.*`). The rover stands still at each one to dig or dump. A failed goal is retried `navigation.max_retries` times, then autonomy returns to `IDLE`.

`NAV_DEPOSIT` and `AUTO` skip the deposition sequence's blind `DRIVE_TO_CONSTRUCTION_ZONE` step, since Nav2 has already put the rover at the berm.

`STOP` and Ctrl-C cancel an active Nav2 goal; otherwise Nav2 would keep driving it.

---

## Excavation

The excavation sequence is implemented in:

```text
qpl_autonomy/excavation.py
```

The sequence performs the following steps:

```text
STOP_WHEELS
    ↓
START_SPIN        (drum forward)
    ↓
LOWER_BUCKET      (spinning)
    ↓
RAISE_BUCKET      (spinning)
    ↓
STOP_BUCKET
    ↓
COMPLETE
```

The rover stands still throughout. The drum starts before the bucket reaches the regolith and keeps spinning until the bucket is back up.

Actuator transitions are based on feedback from:

```text
/joint_states
```

---

## Deposition

The deposition sequence is implemented in:

```text
qpl_autonomy/deposition.py
```

The sequence performs the following steps:

```text
DRIVE_TO_CONSTRUCTION_ZONE   (blind DEPOSIT only)
    ↓
STOP_WHEELS
    ↓
RAISE_BUCKET
    ↓
DUMP_REGOLITH     (drum backwards, dump_duration_s)
    ↓
STOP_BUCKET
    ↓
COMPLETE
```

Blind `DEPOSIT` first drives `drive_to_zone_distance_m` forward, measured with drivetrain odometry. The rover then stands still with the bucket raised and spins the drum backwards. Nothing reports when the drum is empty, so the dump is the one timed step.

Actuator position transitions are based on feedback from:

```text
/joint_states
```

---

## Feedback and Commanding

The autonomy node uses feedback from the rover rather than fixed timing for mechanical sequence progression, except the deposition dump, which is timed.

### Actuator Feedback

Linear actuator positions are read from:

```text
/joint_states
```

The FSM checks both linear actuators against the commanded target position.

A configurable position tolerance determines when the target is considered reached.

### Drivetrain Odometry

Drivetrain odometry is read from:

```text
/diff_cont/odom
```

Blind `DEPOSIT` records the starting odometry position and calculates the distance travelled from the change in `x` and `y`.

This allows the sequences to measure relative movement without requiring global localisation.

### Continuous Commands

Active motion commands are refreshed at a configurable update rate.

This applies to:

* Drivetrain velocity commands
* Drum rotation commands
* Linear actuator position commands while moving toward a target

Continuous publishing is required because the downstream command interfaces use command timeouts/watchdogs.

---

## Command Interfaces

The autonomy node publishes drivetrain commands through:

```text
/cmd_vel_nav
```

These commands are passed through the rover's drivetrain command multiplexer before reaching the drive controller.

Linear actuator commands are published through:

```text
/drum_lift_control/autonomy
```

and drum rotation commands through:

```text
/drum_spin_control/autonomy
```

The autonomy node therefore interfaces with the existing rover control system rather than directly controlling the underlying hardware controllers.

---

## Configuration

Autonomy tuning values are stored in:

```text
config/fsm_tuning.yaml
```

---

## Launching

The autonomy node can be launched using:

```text
launch/autonomy.launch.py
```

The launch file loads:

```text
config/fsm_tuning.yaml
```

and starts:

```text
autonomy_node
```

The node can also be tested by publishing commands directly to:

```text
/autonomy/command
```

For example:

```bash
ros2 topic pub --once /autonomy/command std_msgs/msg/String "{data: 'EXCAVATE'}"
```

```bash
ros2 topic pub --once /autonomy/command std_msgs/msg/String "{data: 'DEPOSIT'}"
```

```bash
ros2 topic pub --once /autonomy/command std_msgs/msg/String "{data: 'STOP'}"
```

The repository also provides shell aliases for these commands where configured.

---

## Development Notes

When modifying `qpl_autonomy`:

* Keep excavation and deposition as independent sequences.
* Use feedback rather than fixed sleeps or timing delays for FSM transitions where feedback exists (the dump has none).
* Keep physical tuning values in `config/fsm_tuning.yaml`.
* Continue publishing active motion commands at the configured update rate.
* Use drivetrain odometry only for relative movement within a sequence.
* Do not add localisation or navigation dependencies to the mechanical FSMs. Navigation lives in `autonomy_node.py` and `navigation.py`, which run a sequence once a waypoint is reached.
* Keep hardware-specific control inside the existing rover control interfaces.
* Treat `STOP` as a normal autonomy abort, separate from the rover's emergency-stop system.
* Chain excavation and deposition only through `AUTO` in `autonomy_node.py`; keep the sequences themselves independent.

The autonomy FSMs are intended to provide reusable mission-level mechanical behaviours. Higher-level navigation or mission logic can determine when and where a sequence should be executed without requiring the FSM itself to manage localisation or navigation.
