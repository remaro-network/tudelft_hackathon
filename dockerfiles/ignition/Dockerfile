FROM ros:foxy-ros-core

RUN apt update && apt install -y --no-install-recommends \
  git \
  wget \
  curl \
  gnupg \
  lsb-release \
  python3-pip \
	&& rm -rf /var/lib/apt/lists/

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends\
  python3-vcstool \
  python3-colcon-common-extensions \
	&& rm -rf /var/lib/apt/lists/

RUN mkdir -p /ign_ws/src
WORKDIR /ign_ws/src
RUN wget https://raw.githubusercontent.com/Rezenders/bluerov2_ignition/main/garden.repos
RUN vcs import < garden.repos

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt update && apt install -y --no-install-recommends\
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ') \
	&& rm -rf /var/lib/apt/lists/

WORKDIR /ign_ws/

RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 2' \
            && colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install --executor sequential"]

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
