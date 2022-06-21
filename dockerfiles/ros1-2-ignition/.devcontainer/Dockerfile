FROM rezenders/ignition:hackathon-nvidia

USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
        && apt-get -y install --no-install-recommends nano ros-foxy-rviz2 terminator gdb \
        && rm -rf /var/lib/apt/lists/
USER docker

RUN echo "alias sf='source /opt/ros/foxy/setup.bash'"  >> /home/docker/.bashrc
RUN echo "alias s='source install/setup.bash'" >> /home/docker/.bashrc
RUN echo "alias m='bash build.sh'" >> /home/docker/.bashrc
RUN echo "source /home/docker/tudelft_hackathon_ws/install/setup.bash"  >> /home/docker/.bashrc
RUN echo "source /ign_ws/install/setup.bash"  >> /home/docker/.bashrc
RUN echo "source /home/docker/ardupilot/Tools/completion/completion.bash"  >> /home/docker/.bashrc

COPY . /home/docker/tudelft_hackathon_ws/src/tudelft_hackathon/
RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 1' \
&& source /opt/ros/foxy/setup.bash \
&& colcon build --symlink-install"]

WORKDIR /home/docker/tudelft_hackathon_ws/
