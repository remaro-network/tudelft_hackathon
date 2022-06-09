FROM ignition:foxy-nvidia

RUN apt update && apt install -y --no-install-recommends \
  rapidjson-dev \
  libboost-all-dev \
  python3-pip \
  python3-vcstool \
  python3-rosdep \
  python-is-python3 \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/

RUN rosdep init

WORKDIR /
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git -b ignition-garden

WORKDIR /ardupilot_gazebo/build
# RUN mkdir build && cd build

RUN [ "/bin/bash","-c","source /ign_ws/install/setup.bash \
            && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo\
            && make"]

ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}
ENV IGN_GAZEBO_RESOURCE_PATH=/ardupilot_gazebo/models:/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}

WORKDIR /
RUN git clone https://github.com/Rezenders/bluerov2_ignition.git
ENV IGN_GAZEBO_RESOURCE_PATH=/bluerov2_ignition/models:/bluerov2_ignition/worlds:${IGN_GAZEBO_RESOURCE_PATH}

WORKDIR /
RUN git clone https://github.com/remaro-network/remaro_worlds.git -b ign-garden
ENV IGN_GAZEBO_RESOURCE_PATH=/remaro_worlds/models:/remaro_worlds/worlds:${IGN_GAZEBO_RESOURCE_PATH}

RUN mkdir -p /tudelft_hackathon_ws/src

WORKDIR /tudelft_hackathon_ws/src
RUN git clone https://github.com/remaro-network/tudelft_hackathon.git

WORKDIR /tudelft_hackathon_ws/
RUN [ "/bin/bash","-c","source /opt/ros/foxy/setup.bash \
            && apt update && rosdep update \
            && rosdep install --from-paths src --ignore-src -r -y \
            && rm -rf /var/lib/apt/lists/"]

RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 1' \
            && source /opt/ros/foxy/setup.bash \
            && colcon build --symlink-install"]

WORKDIR /tudelft_hackathon_ws/

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
