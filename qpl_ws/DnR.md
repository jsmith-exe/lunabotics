# Debugging and Resiliency
A list of commands and processes to try and know when stuff goes wrong.

---
## General
**Clean build**: in the linux environment, run```qpl_clean_build```.
This deletes the previous build output before rebuilding.

**Install packages**: in the linux environment, run ```qpl_packages```.

**Auto restart (NOT IMPLEMENTED)**: Nodes should be setup to automatically restart in launch files:
```python
# Example
test = Node(
    package="demo_nodes_cpp",
    executable="talker",
    respawn=True,
    respawn_delay=2.0
)
```
Any unforeseen crashes should therefore automatically restart.

**Stopping nodes**: To kill a node, the following can be used to list ROS processes.
```bash
qpl_list_processes
```

To kill a node, one of the following can be used:
```bash
# Kill by process name; kills processes containing NODE_NAME
# e.g. pkill -f twist_mux
pkill -f NODE_NAME

# Kill by PID; more precise
kill PID
# Terminate if stuck
kill -9 PID
```

**RQT Log Console**: Run `rqt` in WSL; in the toolbar, Logging > Console.
Log entries can be excluded or highlighted based on rules.
**Pausing will reduce network load; filtering will not.**

---
## Topics and nodes
**Listing**: Run `ros2 node list` and `ros2 topic list` to check if nodes and topics are running.

**Node Graph**: Run `rqt` in WSL; in the toolbar, Introspection > Node Graph.
Hover over certain checkboxes to see how they impact the graph.

**Topic monitor**: Run `rqt` in WSL; in the toolbar, Topics > Topic Monitor.
Sometimes topics can be available but not be published to; this can help check, due to the frequency being recorded.

**Image View**: Run `rqt` in WSL; in the toolbar, Plugins > Visualization > Image View.
Uncompressed or large images can fail to be sent.

**TF inspection**: Install `rqt_tf_tree`, run `rqt --force-discover` (force discovery only needed for first run);
in the toolbar, Plugins > Visualization > TF Tree.
```bash
# Install the package for rqt_tf_tree
sudo apt update
sudo apt install ros-humble-rqt-tf-tree

# Run RQT
rqt --force-discover

# Directly open if needed (probably not)
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```


---
## Networking
**Ping the target device**: use the `ping` command to check if the target device is reachable.
Note that ping can fail yet the device can still be reachable, if the device just has ICMP disabled.

**Restart ROS Daemon**: The ROS daemon discovers nodes and topics; restarting can force discovery.
```bash
ros2 daemon stop && ros2 daemon start
```

**Test multicast**: ROS2 uses multicast for discovery by default; if this is unsupported, discovery will fail.
```bash
# On a device, start listening for UDP multicast:
ros2 multicast receive
# On the other device, send a multicast message:
ros2 multicast send
```

**Demo node**: Run a minimal node to check if the basic ROS2 communication is working.
Some nodes can be misconfigured or have network issues specific to the nature of the data (e.g., images being too large).
```bash
# Install if not already installed:
sudo apt-get install ros-humble-demo-nodes-cpp
# On one device, run the talker:
ros2 run demo_nodes_cpp talker
# On the other device, run the listener:
ros2 run demo_nodes_cpp listener
```

---
## Camera
**List video feeds**:
```bash
v4l2-ctl --list-devices
```

**Take a picture**:
```bash
sudo ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 test.jpg
```

**Theora warning**: RViz2 may show the following warning and Theora compressed video may not come through, something like this:
```
[TheoraSubscriber]: [theora] Packet was not a Theora header
```
Restart the camera node if this happens.
