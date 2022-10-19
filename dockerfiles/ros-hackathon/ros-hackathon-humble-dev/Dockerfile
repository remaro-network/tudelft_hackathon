FROM rezenders/ros-hackathon-humble

COPY . /tudelft_hackathon_ws/src/tudelft_hackathon/
RUN [ "/bin/bash","-c","source /opt/ros/humble/setup.bash \
&& colcon build --symlink-install"]

COPY dockerfiles/ros-hackathon/ros-hackathon-humble-dev/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
