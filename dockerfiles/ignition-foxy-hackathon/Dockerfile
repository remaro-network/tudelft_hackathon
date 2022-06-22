FROM rezenders/ignition:foxy-ardusub

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

RUN rosdep init

USER docker

WORKDIR /home/docker/
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git -b ignition-garden

WORKDIR /home/docker/ardupilot_gazebo

RUN [ "/bin/bash","-c","source /ign_ws/install/setup.bash \
            && mkdir build && cd build \
            && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo\
            && make"]

ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/home/docker/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}
ENV IGN_GAZEBO_RESOURCE_PATH=/home/docker/ardupilot_gazebo/models:/home/docker/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}

RUN mkdir -p /home/docker/tudelft_hackathon_ws/src

WORKDIR /home/docker/tudelft_hackathon_ws/
RUN wget https://raw.githubusercontent.com/remaro-network/tudelft_hackathon/ros2/hackathon.rosinstall
RUN vcs import src < hackathon.rosinstall --recursive


USER root

WORKDIR /home/docker/tudelft_hackathon_ws/src/mavros/mavros/scripts
RUN [ "/bin/bash","-c","apt update \
            && ./install_geographiclib_datasets.sh \
            && rm -rf /var/lib/apt/lists/"]

WORKDIR /home/docker/tudelft_hackathon_ws/
RUN [ "/bin/bash","-c","source /opt/ros/foxy/setup.bash \
            && apt update && rosdep update \
            && rosdep install --from-paths src --ignore-src -r -y \
            && rm -rf /var/lib/apt/lists/"]

USER docker
RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 1' \
            && source /opt/ros/foxy/setup.bash \
            && colcon build --symlink-install"]

ENV IGN_GAZEBO_RESOURCE_PATH=/home/docker/tudelft_hackathon_ws/src/bluerov2_ignition/models:/home/docker/bluerov2_ignition/worlds:${IGN_GAZEBO_RESOURCE_PATH}
ENV IGN_GAZEBO_RESOURCE_PATH=/home/docker/tudelft_hackathon_ws/src/remaro_worlds/models:/home/docker/tudelft_hackathon_ws/src/remaro_worlds/worlds:${IGN_GAZEBO_RESOURCE_PATH}

WORKDIR /home/docker/tudelft_hackathon_ws/

COPY entrypoint.sh /home/docker/
ENTRYPOINT ["/home/docker/entrypoint.sh"]
CMD ["bash"]
