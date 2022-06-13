FROM rezenders/ignition:foxy-nvidia

ENV ROS_DISTRO=foxy
ENV AMENT_PREFIX_PATH=/opt/ros/foxy
ENV COLCON_PREFIX_PATH=/opt/ros/foxy
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib
ENV PATH=/opt/ros/foxy/bin:$PATH
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
ENV DEBIAN_FRONTEND=


ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \ 
        && apt-get -y install --no-install-recommends nano ros-foxy-rviz2 terminator gdb

# play ros1 bags with ros2 bag play
RUN apt-get -y install ros-foxy-rosbag2-bag-v2-plugins
RUN apt-get -y install --no-install-recommends ~nros-foxy-rqt*

# build tools ros1 catkin
RUN apt-get -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools python3-osrf-pycommon

# navigation stack dependencies Huy
# RUN apt-get -y install autoconf ros-noetic-dynamic-reconfigure ros-noetic-mavros ros-noetic-cmake-modules libyaml-cpp-dev ros-noetic-tf2-geometry-msgs

#-> Linear algebra library
RUN echo "Installing Eigen 3 last version ..." && \
    apt-get install -y libeigen3-dev

# Set up auto-source of workspace for ros user
ARG WORKSPACE

# shortcuts for convenience ;)
# please customize to your liking
RUN echo "alias sn='source /opt/ros/noetic/setup.bash'" >> /home/ros/.bashrc
RUN echo "alias sf='source /opt/ros/foxy/setup.bash'"  >> /home/ros/.bashrc
RUN echo "alias s='source install/setup.bash'" >> /home/ros/.bashrc
RUN echo "alias m='bash build.sh'" >> /home/ros/.bashrc

# source ros2 workspace at startup of container
RUN echo "if [ -f ${WORKSPACE}/ros2_ws/install/setup.bash ]; then source ${WORKSPACE}/ros2_ws/install/setup.bash; fi" >> /home/ros/.bashrc

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

COPY . /tudelft_hackathon_ws/src/tudelft_hackathon/
RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 1' \
&& source /opt/ros/foxy/setup.bash \
&& colcon build --symlink-install"]