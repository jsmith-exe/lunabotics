# Basestation
Functionality for parsing data from the rover and sending commands should be kept here.

## Current functionality
Keypresses and PlayStation dualsense controller inputs are parsed and converted into JSON messages
that can be converted to ROS Twist messages. These are sent via TCP to a ROS publisher, which publishes
the Twist messages.
> The reason for this convoluted structure is to gather input at the native OS level (which may be Windows)
> and still use ROS publishers, maintaining consistent architecture and granting the functional benefits of ROS.

**Keyboard should be cross-platform. PlayStation controller will only work on Windows.**

## Code structure
The core source code is in basestation (luna_ws/src/basestation/basestation). This is further split into the following modules:
- **controls**: contains code for parsing keypresses and controller inputs and converting them into messages suitable for publishing.
  - **controllers**: contains code for parsing controller inputs.
  - **local_turtle_test.py**: contains code for testing controls directly (without any forwarding or ROS).
- **forwarding**: contains TCP forwarding code that sends messages to the ROS publisher.
- **nodes**: contains ROS nodes that the basestation uses.
- **main.py**: Runs on the native OS to gather input and send it to the ROS publisher.

## How to use
The following assumes you know how to build and install ROS packages.
A Python environment is needed, ideally Python 3.10. The following instructions assume a Windows OS, but the commands should be adaptable.
1. If you don't have it installed, install Python 3.10. I would use uv:
```commandline
uv python install 3.10
```
2. In luna_ws/src/basestation, create a virtual environment, activate it, and install 
the requirements - if you're not using uv, omit the uv part:
```commandline
python3.10 -m venv venv
venv\Scripts\activate
uv pip install -r .\requirements.txt
```

To run:
1. Ensure you've activated your environment. You might see a (venv) prefix
in your terminal to indicate this. If not, run the following command in the same folder as the venv folder:
```commandline
venv\Scripts\activate
```
2. Run the controls publisher in a linux environment; this was tested in WSL.
```commandline
ros2 run basestation nav_pub
```
3. Run main.py; inputs will now be forwarded to the ROS publisher.
```commandline
python .\basestation\main.py
```
4. On Windows, you will likely need to turn off your local network firewall.
   1. Search and open 'Windows Security'
   2. Firewall and network protection
   3. Private network
   4. Turn off 'Windows Defender Firewall'
Turn it back on when finished.
5. The publisher can be tested by running the nav_sub:
```commandline
ros2 run basestation nav_sub
```

---
#### Docker documentation
I had planned to support MacOS via Docker, but ran into network issues.
For future reference, I'm documenting how I intended the Docker files to be used; these probably don't work.

Docker desktop should be installed.

The docker-compose.yaml file should contain the configuration needed to run the docker container, by running the command:
```commandline
docker compose up --build
```
This will automatically build the image from the Dockerfile and run the image.
