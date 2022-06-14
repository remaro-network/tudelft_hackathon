#!/bin/bash
pip install future
pip3 install mavproxy pymavlink

cd ardupilot_gazebo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
cd ..
cd ..
echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$PWD/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export IGN_GAZEBO_RESOURCE_PATH=$PWD/ardupilot_gazebo/models:$PWD/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}' >> ~/.bashrc

echo 'export IGN_GAZEBO_RESOURCE_PATH=$PWD/ros2_ws/src/bluerov2_ignition/models:$PWD/ros2_ws/src/bluerov2_ignition/worlds:${IGN_GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
echo 'export IGN_GAZEBO_RESOURCE_PATH=$PWD/ros2_ws/src/remaro_worlds/models:$PWD/ros2_ws/src/remaro_worlds/worlds:${IGN_GAZEBO_RESOURCE_PATH}' >> ~/.bashrc

cd ardupilot
git checkout ArduSub-stable -b new-branch
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf clean
./waf configure --board Pixhawk1
./waf sub
# Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --out=udp:0.0.0.0:14550 --console