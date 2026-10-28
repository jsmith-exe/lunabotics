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
│   ├── nav/
│   │   └── nav_autonomy.yaml
│   └── legacy/
├── launch/
│   ├── autonomy.launch.py
│   ├── nav/
│   │   └── nav_autonomy.launch.py
│   └── legacy/
├── qpl_autonomy/
│   ├── __init__.py
│   ├── autonomy_node.py
│   ├── excavation.py
│   ├── deposition.py
│   ├── nav/
│   │   ├── __init__.py
│   │   ├── nav_autonomy_node.py
│   │   ├── nav_excavation.py
│   │   └── nav_deposition.py
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

### Variants

Two alternative nodes run the same excavation and deposition sequences but measure the drive stages differently. The `mode` launch argument selects which node runs, and all of them read their tuning from `config/fsm_tuning.yaml`.

| `mode`           | Node                      | Drive stages end when...                                                 |
| ---------------- | ------------------------- | ------------------------------------------------------------------------ |
| `odom` (default) | `autonomy_node`           | Drivetrain odometry (`/diff_cont/odom`) reaches the `*_distance_m` value |
| `localised`      | `localised_autonomy_node` | The `map -> base_footprint` TF reaches the `*_distance_m` value          |
| `timed`          | `timed_autonomy_node`     | The `*_duration_s` value has elapsed                                     |

```bash
ros2 launch qpl_autonomy autonomy.launch.py mode:=timed
```

`localised_autonomy_node` requires the localisation stack to be running. It refuses to start a sequence without a pose, and stops the active sequence if the pose is lost or older than `localisation.max_pose_age_s`.

The variants subclass `AutonomyNode` and the sequence classes, so the original node and FSMs are unchanged.

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

## Nav Autonomy

`nav_autonomy_node` uses Nav2 to drive between the excavation zone and the berm. It is a separate node from the blind autonomy above. The blind node, its variants, their sequences and `fsm_tuning.yaml` are not used or changed by it, so blind autonomy remains as a backup.

It is started by `qpl_nav` on the rover or `qpl_nav_sim` against the sim (`qpl_rover` `navigation_launch.py`, which includes `launch/nav/nav_autonomy.launch.py` and passes on `use_sim_time`). It then sits in `IDLE` until it receives a command, so nothing moves just because `qpl_nav` is running. To launch Nav2 without it, use `qpl_nav nav_autonomy:=false`.

Commands are `std_msgs/msg/String` on `/nav_autonomy/command`:

| Command        | Basestation button         | Alias              | Function                                                            |
| -------------- | -------------------------- | ------------------ | ------------------------------------------------------------------- |
| `NAV_EXCAVATE` | Nav Excavate               | `qpl_nav_excavate` | Drive to the excavation waypoint with Nav2, then excavate           |
| `NAV_DEPOSIT`  | Nav Deposit                | `qpl_nav_deposit`  | Drive to the deposition waypoint with Nav2, then deposit            |
| `AUTO`         | Full Autonomy (Low Risk)   | `qpl_full_auto`         | Full autonomy: repeat `NAV_EXCAVATE` then `NAV_DEPOSIT` until `STOP` |
| `AUTO_TRENCH`  | High Risk (Trench)         | `qpl_full_auto_trench`  | Full autonomy digging one fixed trench every cycle (see below)      |
| `AUTO_BOULDERS` | Full Autonomy (Boulders Only) | `qpl_full_auto_boulders` | `AUTO`, avoiding boulders only (see below)                      |
| `AUTO_TRENCH_BOULDERS` | High Risk (Boulders Only) | `qpl_full_auto_trench_boulders` | `AUTO_TRENCH`, avoiding boulders only (see below)      |
| `STOP`         | Stop                       | `qpl_full_auto_stop`    | Cancel the Nav2 goal or active sequence and return to `IDLE`        |

The buttons are in the basestation's **Nav Autonomy** section. Like the blind autonomy buttons, they only work while teleop is disabled, because teleop has priority over Nav2 in the drive mux. Either Stop button in the basestation stops both autonomy nodes.

### Full autonomy

```text
navigate to excavation waypoint -> excavate -> navigate to deposition waypoint -> deposit -> repeat
```

The rover stands still while digging and dumping, which is why it is labelled low risk:

* **Excavation:** stop -> spin drum forward -> lower bucket -> hold it down for `dig_duration_s` -> raise bucket -> stop spinning.
* **Deposition:** stop -> bucket raised -> spin drum backwards for `dump_duration_s` -> stop spinning.

Nothing reports when the bucket is full or empty, so the dig and dump are timed (30 s and 60 s). The rest moves on actuator feedback.

### High risk: trench

`AUTO_TRENCH` is the same loop, but instead of standing still at a dig spot the rover digs one fixed trench, the same every cycle:

```text
navigate to trench.start_xy -> turn to face away from trench.end_xy (ALIGNING) -> spin drum, lower bucket
-> reverse to trench.end_xy with the bucket down -> raise bucket, stop spinning -> re-localise -> deposit as in AUTO -> repeat
```

* It only digs reversing: the drum can't dig driving forwards. It reverses at `trench.speed` (0.05 m/s), steering back onto the line between the two points.
* It stops the trench early, raises the bucket and carries on to the berm if something is behind it in the costmap, or after twice the time the trench should take.
* While it runs, the depth filters don't mark craters within 0.8 m of the trench (`/nav_autonomy/trench_corridor`), or last cycle's trench would block the way back to its start. Boulders there are still marked. Rulebook craters in the corridor are not avoided, which is part of the risk.
* The trench is `trench.start_xy` -> `trench.end_xy` in `config/nav/nav_autonomy.yaml`: (4.4, 1.5) to (6.9, 1.5), along the south row of dig spots, ending nearest the berm.

### Boulders only

`AUTO_BOULDERS` and `AUTO_TRENCH_BOULDERS` are `AUTO` and `AUTO_TRENCH`, but they avoid only boulders and drive through craters. They are for a layout where craters close every way to the berm with no gap wide enough for the rover. While one runs, nav autonomy sets `/nav_autonomy/ignore_craters` and the depth filters stop marking craters anywhere; craters they remembered before (the 360) come back once it ends. The rover can drop a wheel into a crater, so use them only when the normal modes can't get through.

### Navigation

* It needs Nav2 to be running and the rover to be localised from the AprilTag. If Nav2 is not up, or the rover isn't localised (`localisation.*` in the config: a recent tag fix that the global EKF agrees with, and little driving since), the command is refused.
* Dig and dump spots are lists of map-frame `[x, y, yaw]` candidates in `config/nav/nav_autonomy.yaml` (`navigation.excavation_candidates`, `navigation.deposition_candidates`). Each leg takes the next candidate whose footprint is clear in the global costmap, and switches if it becomes blocked on the way. Dig spots are used in turn; there is one dump spot, on the berm. Dig and dump spots are approached head-on, in any heading: the drum is under the middle of the rover. The dig or dump starts once the rover is within `arrival_radius_m` (0.25 m) of the candidate, or when Nav2 reports the goal reached. That small buffer stops it shuffling to correct itself.
* In AUTO, after each dig the rover re-localises (state `RELOCALISING`): it turns on the spot until the front camera (or the rear one, within 2.5 m) faces the tag, and waits for 5 fixes and the global EKF agreeing with them before driving to the berm. It is skipped when the tag is more than 4.4 m away or under 20° off its wall (too far for the cameras in the sim), stops turning if the next step would put the footprint on an obstacle, and gives up after 25 s. Settings: `relocalise.*` in `config/nav/nav_autonomy.yaml`.
* In AUTO, after each dump the rover backs 1 m straight out of the berm, the way it drove in (state `BACKING_OUT`, `deposition.back_out_m`) before heading for the next dig spot, so it has room to turn away from the east wall. It is skipped if the costmap shows something behind it.
* While navigating, the bucket is held at `travel_lift_position_m` so it does not plough the regolith.
* Each leg goes via waypoints about `route_waypoint_spacing_m` apart along the widest way round in the global costmap (`nav/route.py`), recomputed every 3 s, rather than straight to Nav2's shortest path that fits. If no way round keeps `route_min_clearance_m` from obstacles, the goal goes straight to Nav2.
* A failed goal is retried `max_retries` times. Before each retry the rover turns or drives a short way (`unstick_m`) to a pose with nothing lethal under its footprint, and from the first retry to the end of the leg it is in tight mode: the depth filters shrink the margins round obstacles (`grow_*_tight_m` in `qpl_rover/config/depth_obstacle_filter.yaml`, via `/nav_autonomy/tight`) and Nav2 runs at `tight_speed_percent`. A failed leg to a dig spot moves on to the next clear one. After the last retry, autonomy (including `AUTO`) returns to `IDLE`.
* `STOP` and Ctrl-C cancel the active Nav2 goal; otherwise Nav2 would keep driving it.

All nav autonomy tuning is in `config/nav/nav_autonomy.yaml`, under the node name `nav_autonomy`.

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
