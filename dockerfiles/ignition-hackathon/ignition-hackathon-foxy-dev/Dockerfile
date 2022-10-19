FROM rezenders/ignition:hackathon-nvidia

COPY . /home/docker/tudelft_hackathon_ws/src/tudelft_hackathon/
RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 1' \
            && source /opt/ros/foxy/setup.bash \
            && colcon build --symlink-install"]

WORKDIR /home/docker/tudelft_hackathon_ws/

COPY dockerfiles/ignition-foxy-hackathon-dev/entrypoint.sh /home/docker/
ENTRYPOINT ["/home/docker/entrypoint.sh"]
CMD ["bash"]
