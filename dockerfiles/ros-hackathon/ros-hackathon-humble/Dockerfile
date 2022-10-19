FROM rezenders/ignition:humble

RUN apt update && apt install -y --no-install-recommends\
  git \
  wget \
  build-essential \
  libboost-all-dev \
  python3-pip \
  python3-vcstool \
  python3-rosdep \
  python-is-python3 \
  python3-colcon-common-extensions \
	&& rm -rf /var/lib/apt/lists/

RUN rosdep init

RUN mkdir -p /tudelft_hackathon_ws/src

WORKDIR /tudelft_hackathon_ws

RUN wget https://raw.githubusercontent.com/remaro-network/tudelft_hackathon/ros2/hackathon.rosinstall
RUN vcs import src < hackathon.rosinstall --recursive

RUN [ "/bin/bash","-c","source /opt/ros/humble/setup.bash \
            && apt update && rosdep update \
            && rosdep install --from-paths src --ignore-src -r -y \
            && rm -rf /var/lib/apt/lists/"]

RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 2' \
            && source /opt/ros/humble/setup.bash \
            && colcon build --symlink-install"]

WORKDIR /tudelft_hackathon_ws/src/mavros/mavros/scripts
RUN [ "/bin/bash","-c","apt update \
            && ./install_geographiclib_datasets.sh \
            && rm -rf /var/lib/apt/lists/"]

WORKDIR /tudelft_hackathon_ws/
ENV GZ_SIM_RESOURCE_PATH=/tudelft_hackathon_ws/src/remaro_worlds/models:/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
