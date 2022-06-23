FROM rezenders/ros-foxy-hackathon

COPY . /tudelft_hackathon_ws/src/tudelft_hackathon/
RUN [ "/bin/bash","-c","source /opt/ros/foxy/setup.bash \
&& colcon build --symlink-install"]

COPY dockerfiles/ros-foxy-hackathon-dev/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
