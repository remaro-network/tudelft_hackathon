FROM ros:humble-ros-core-jammy

RUN apt update && apt install -y --no-install-recommends \
  wget \
  gnupg \
  lsb-release \
	&& rm -rf /var/lib/apt/lists/

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
  gz-garden \
	&& rm -rf /var/lib/apt/lists/

CMD ["bash"]
