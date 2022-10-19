FROM rezenders/ignition:hackathon-nvidia-humble

COPY . /home/docker/tudelft_hackathon_ws/src/tudelft_hackathon/
RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 1' \
            && source /opt/ros/humble/setup.bash \
            && colcon build --symlink-install"]

WORKDIR /home/docker/tudelft_hackathon_ws/

COPY dockerfiles/ignition-hackathon/ignition-hackathon-humble-dev/entrypoint.sh /home/docker/
ENTRYPOINT ["/home/docker/entrypoint.sh"]
CMD ["bash"]
