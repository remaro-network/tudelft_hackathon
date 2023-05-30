ARG BASE_IMAGE="ghcr.io/remaro-network/ignition-ardusub-humble:latest"
FROM $BASE_IMAGE

USER root
RUN apt update && apt install -y --no-install-recommends \
  vim \
  rapidjson-dev \
  libboost-all-dev \
  python3-pip \
  python3-vcstool \
  python3-rosdep \
  python-is-python3 \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/

USER docker

WORKDIR /home/docker/
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git

WORKDIR /home/docker/ardupilot_gazebo

RUN [ "/bin/bash","-c","mkdir build && cd build \
            && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo\
            && make"]

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/docker/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
ENV GZ_SIM_RESOURCE_PATH=/home/docker/ardupilot_gazebo/models:/home/docker/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

RUN mkdir -p /home/docker/tudelft_hackathon_ws/src

WORKDIR /home/docker/tudelft_hackathon_ws/
RUN wget https://raw.githubusercontent.com/remaro-network/tudelft_hackathon/ros2/hackathon.rosinstall
RUN vcs import src < hackathon.rosinstall --recursive

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
      && sudo apt update \
      && sudo rosdep init \
      && rosdep update \
      && rosdep install --from-paths src --ignore-src -r -y \
      && sudo rm -rf /var/lib/apt/lists/"]

RUN ["/bin/bash", "-c", "sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh"]

ENV GZ_VERSION="garden"
RUN [ "/bin/bash","-c","source /opt/ros/humble/setup.bash \
            && colcon build --symlink-install"]

ENV GZ_SIM_RESOURCE_PATH=/home/docker/tudelft_hackathon_ws/src/bluerov2_ignition/models:/home/docker/bluerov2_ignition/worlds:${GZ_SIM_RESOURCE_PATH}
ENV GZ_SIM_RESOURCE_PATH=/home/docker/tudelft_hackathon_ws/src/remaro_worlds/models:/home/docker/tudelft_hackathon_ws/src/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}

WORKDIR /home/docker/tudelft_hackathon_ws/

COPY entrypoint.sh /home/docker/
ENTRYPOINT ["/home/docker/entrypoint.sh"]
CMD ["bash"]
