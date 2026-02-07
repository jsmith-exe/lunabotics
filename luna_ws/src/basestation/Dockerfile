FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /lunabotics_ws

# TODO Optimise to only copy necessary files for dependency installation
COPY . .

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN colcon build

COPY docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh && \
    sed -i 's/\r$//' /docker_entrypoint.sh # Remove Windows line endings

# TODO Set up entrypoint to source ROS2 and workspace setup files

#CMD ["python3", "/lunabotics_ws/src/basestation/basestation/forwarding/tcp_receiver.py"]
CMD ["/docker_entrypoint.sh", "ros2", "run", "basestation", "pub"]
#CMD ["sleep", "infinity"]