# qpl_autonomy

ROS 2 package containing the mission-level autonomy logic for the QPL rover, including feedback-based excavation and deposition sequences, configuration, and launch files.

## Overview

`qpl_autonomy` provides the rover's high-level mechanical autonomy sequences.

The package currently contains two independent finite state machines (FSMs):

* Excavation
* Deposition

Each sequence is started independently through a command received from the Basestation. The sequences do not automatically transition into one another.

The autonomy logic is designed to operate without requiring localisation or navigation. It uses actuator feedback and drivetrain odometry to determine when each step of a sequence has completed.

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

| Command    | Function                                               |
| ---------- | ------------------------------------------------------ |
| `EXCAVATE` | Start the excavation sequence                          |
| `DEPOSIT`  | Start the deposition sequence                          |
| `STOP`     | Stop the active sequence and return autonomy to `IDLE` |

Only one sequence can be active at a time.

When autonomy is idle, an `EXCAVATE` or `DEPOSIT` command starts the corresponding sequence. A sequence must complete before autonomy returns to `IDLE`.

##### `STOP` does not start another sequence or attempt to reposition the rover. It stops autonomous motion and leaves the rover in its current state.

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
LOWER_TO_CONTACT
    ↓
LOWER_TO_MAX
    ↓
DRIVE_FORWARD
    ↓
STOP_WHEELS_AFTER_DRIVE
    ↓
LIFT_BUCKET
    ↓
STOP_BUCKET
    ↓
COMPLETE
```

The bucket drum is operated during the relevant excavation stages while the linear actuators are commanded to configured positions.

The excavation drive distance is measured using drivetrain odometry rather than a timed delay.

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
DRIVE_TO_CONSTRUCTION_ZONE
    ↓
LIFT_BUCKET
    ↓
DEPOSIT_REGOLITH
    ↓
STOP_WHEELS
    ↓
STOP_BUCKET
    ↓
RETURN_TO_NORMAL_HEIGHT
    ↓
COMPLETE
```

The rover first drives the configured distance to the construction zone, then raises the bucket and operates the drum while slowly driving forward.

The drive distances are measured using drivetrain odometry.

Actuator position transitions are based on feedback from:

```text
/joint_states
```

---

## Feedback and Commanding

The autonomy node uses feedback from the rover rather than fixed timing for mechanical sequence progression.

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

The FSM records the starting odometry position for each relevant drive stage and calculates the distance travelled from the change in `x` and `y`.

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
* Use feedback rather than fixed sleeps or timing delays for FSM transitions.
* Keep physical tuning values in `config/fsm_tuning.yaml`.
* Continue publishing active motion commands at the configured update rate.
* Use drivetrain odometry only for relative movement within a sequence.
* Do not add localisation or navigation dependencies to the mechanical FSMs.
* Keep hardware-specific control inside the existing rover control interfaces.
* Treat `STOP` as a normal autonomy abort, separate from the rover's emergency-stop system.
* Avoid automatically chaining excavation and deposition sequences.

The autonomy FSMs are intended to provide reusable mission-level mechanical behaviours. Higher-level navigation or mission logic can determine when and where a sequence should be executed without requiring the FSM itself to manage localisation or navigation.
