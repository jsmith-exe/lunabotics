FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /lunabotics_ws

COPY . .

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN colcon build

RUN echo "source ../ros_entrypoint.sh" >> ~/.bashrc && \
    echo "source install/setup.bash" >> ~/.bashrc

CMD ["sleep", "infinity"]