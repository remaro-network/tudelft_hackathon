

# steps for running ros1 bridge
echo "-- source noetic setup"
. /opt/ros/noetic/setup.bash

echo "-- source foxy setup"
. /opt/ros/foxy/setup.bash

echo "-- export master uri"
export ROS_MASTER_URI=http://localhost:11311

echo "-- run bridge"
ros2 run ros1_bridge dynamic_bridge