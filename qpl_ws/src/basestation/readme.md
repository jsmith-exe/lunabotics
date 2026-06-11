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

## Prerequisites
Verify you can run powershell in wsl.
```bash
powershell.exe
```

If it fails, the following steps copied from [a comment on reddit](https://www.reddit.com/r/bashonubuntuonwindows/comments/11vx61n/wsl2_error_cannot_execute_binary_file_exec_format/) may help.

If successful, you will likely want to disable powershell execution restrictions. In powershell, run:
```powershell
Set-ExecutionPolicy Unrestricted
```
This will allow you to run any powershell script, at the cost of disabling a safeguard.
It means you can run stuff that might not be safe, but that's just Tuesday.

---
1. Create binformat config file in WSL (https://github.com/microsoft/WSL/issues/8986#issuecomment-1332413859)
```bash
sudo nano /usr/lib/binfmt.d/WSLInterop.conf
```
With contents:
```
:WSLInterop:M::MZ::/init:PF
```
2. install binformat systemd support
```bash
sudo apt update
sudo apt install binfmt-support
```

3. Restart binformat related systemd services (https://github.com/microsoft/WSL/issues/8986#issuecomment-1332452012)
```bash
sudo systemctl restart systemd-binfmt
sudo systemctl restart binfmt-support
```

4. Verify: run `sudo ls -Fal /proc/sys/fs/binfmt_misc`, you should see something like:
```
total 0
drwxr-xr-x 2 root root 0 Mar 24 11:11 ./
dr-xr-xr-x 1 root root 0 Mar 24 11:11 ../
-rw-r--r-- 1 root root 0 Mar 24 11:35 WSLInterop
-rw-r--r-- 1 root root 0 Mar 24 11:35 jar
-rw-r--r-- 1 root root 0 Mar 24 11:35 python3.11
--w------- 1 root root 0 Mar 24 11:35 register
-rw-r--r-- 1 root root 0 Mar 24 11:35 status
```
And run: `sudo cat /proc/sys/fs/binfmt_misc/WSLInterop`, should see something like:
```
enabled
interpreter /init
flags: PF
offset 0
magic 4d5a
```

## Setup
> Note: you can just run the following in powershell:
> ```powershell
> .\setup_controller.ps1
> ```
> In the ```lunabotics\qpl_ws\src\basestation``` directory.
> This is untested.

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

## Running the controller
> Note: you can just run the following in powershell:
> ```powershell
> .\run_controller.ps1
> ```
> In either the ```lunabotics``` directory or ```lunabotics\qpl_ws\src\basestation```.

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
4. On Windows, you may need to turn off your local network firewall.
   1. Search and open 'Windows Security'
   2. Firewall and network protection
   3. Private network
   4. Turn off 'Windows Defender Firewall'
Turn it back on when finished.
5. The publisher can be tested by running the nav_sub on device running ROS on the local network:
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
