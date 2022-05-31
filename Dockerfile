FROM ros:foxy-ros-core

RUN apt update && apt install -y \
  git \
  libboost-all-dev \
  python3-pip \
  python3-vcstool \
  python3-rosdep \
  python3-colcon-common-extensions \
	&& rm -rf /var/lib/apt/lists/

RUN rosdep init

RUN mkdir -p /tudelft_hackathon_ws/src
COPY hackathon.rosinstall /tudelft_hackathon_ws/hackathon.rosinstall

WORKDIR /tudelft_hackathon_ws
RUN vcs import src < hackathon.rosinstall --recursive

RUN [ "/bin/bash","-c","source /opt/ros/foxy/setup.bash \
            && apt update && rosdep update \
            && rosdep install --from-paths src --ignore-src -r -y \
            && rm -rf /var/lib/apt/lists/"]

RUN [ "/bin/bash","-c","source /opt/ros/foxy/setup.bash \
            && colcon build --symlink-install"]

WORKDIR /tudelft_hackathon_ws/src/mavros/mavros/scripts
RUN [ "/bin/bash","-c","apt update \
            && ./install_geographiclib_datasets.sh \
            && rm -rf /var/lib/apt/lists/"]

WORKDIR /tudelft_hackathon_ws/

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
