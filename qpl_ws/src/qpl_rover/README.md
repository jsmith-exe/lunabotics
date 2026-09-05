# qpl_rover

ROS 2 package containing the core software, configuration, launch files, simulation assets, and localisation components for the QPL rover.

## Overview

`qpl_rover` brings together the main components required to run the rover in simulation and on the physical platform, including:

* Rover description and TF configuration
* Sensor and camera configuration
* Localisation
* Navigation-related configuration
* Simulation worlds and arena definitions
* Launch files
* Arena-specific configuration

The package is structured so that environment-specific and hardware-specific parameters can be changed through configuration without modifying the underlying nodes where possible.

## Package Structure

```text
qpl_rover/
├── config/
│   ├── arena/
│   └── localisation/
├── launch/
│   ├── components/
│   └── hardware/
├── qpl_rover/
├── worlds/
└── ...
```

### `config/`

Configuration for the rover and its operating environment.

Arena-specific configuration is stored under:

```text
config/arena/
```

while localisation-specific configuration is stored under:

```text
config/localisation/
```

The active arena can be selected through:

```text
config/arena/selector.yaml
```

### `launch/`

Launch files for starting individual components and complete rover subsystems.

### `qpl_rover/`

Python nodes belonging to the package.

### `worlds/`

Simulation environments for the supported arenas.

---

## Localisation

The rover's localisation stack combines the available sensor sources to estimate the rover pose in the global `map` frame.

The package includes AprilTag-based visual localisation as part of this system.

### AprilTag Localisation

AprilTags provide an absolute visual reference for rover localisation. The `apriltag_observer` node detects tags from the rover's cameras, estimates the camera-to-tag pose, and uses the TF tree together with the configured arena geometry to calculate the rover pose.

The relevant configuration is split between:

```text
config/localisation/apriltag.yaml
config/arena/
```

This separates detector parameters, such as tag family and physical size, from arena-specific information such as tag placement and arena dimensions.

---

## Arena Configuration

Arena configuration is stored in:

```text
config/arena/
```

The configuration defines properties of the physical environment, including arena dimensions and AprilTag placement.

The active arena configuration is selected using:

```yaml
arena: us
```

in:

```text
config/arena/selector.yaml
```

### Arena and Simulation World Selection

**The arena configuration and simulation world are currently decoupled.**

`selector.yaml` determines which arena configuration is used by the localisation stack. It does **not** determine which simulation world is loaded.

When running in simulation, the arena must therefore be selected in two separate places:

| Purpose                            | File                         | Example                 |
| ---------------------------------- | ---------------------------- | ----------------------- |
| Localisation / arena configuration | `config/arena/selector.yaml` | `arena: us`             |
| Simulation world                   | `launch/sim.launch`          | `worlds/us_arena.world` |

These selections must be kept consistent. Changing only one can result in the localisation stack using the configuration for one arena while the simulator loads a different arena.

| Arena | `selector.yaml` | `sim.launch`     |
| ----- | --------------- | ---------------- |
| UK    | `arena: uk`     | `uk_arena.world` |
| US    | `arena: us`     | `us_arena.world` |

Simulation worlds are stored under:

```text
worlds/
```

---

## Localisation Launch

The localisation system is launched through:

```text
launch/components/map_localisation.launch.py
```

This launch file brings together the relevant localisation components and loads the selected arena configuration.

The arena selected in `config/arena/selector.yaml` is used by the localisation stack, including the AprilTag localisation configuration.

`map_localisation.launch.py` does **not** select the simulation world. The simulation world is selected separately in `launch/sim.launch`.

---

## Development Notes

When adding functionality to `qpl_rover`:

* Prefer configuration over hard-coded physical parameters.
* Keep arena-specific values in `config/arena/`.
* Keep component-specific parameters in the appropriate configuration directory.
* Use the existing TF tree for rover and sensor geometry where possible.
* Keep simulation environments consistent with their corresponding configuration.
* Avoid duplicating physical dimensions or transforms across nodes.

For component-specific documentation, additional README files can be added within the relevant subdirectory as the package grows.
