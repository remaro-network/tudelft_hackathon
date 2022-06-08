# tudelft_hackathon
Repository for the TU Delft hackathon

## Setup

Tested with:
- Ubuntu 20.04
- ROS2 foxy
- Ignition garden (on [this](https://github.com/gazebosim/gz-sim/pull/1402) patch)  
- ArduPilot (Sub-4.1)(14e6dcdc2fa85c1d6f8d298591c0f103de56b4cd)
- ardupilot_gazebo plugin with [this version](https://github.com/ArduPilot/ardupilot_gazebo/tree/aaffdc02580980a17f7717e32e520747051811f3)
- [mavros2](https://github.com/mavlink/mavros)
- Ignition garden version of [remaro_world](https://github.com/remaro-network/remaro_worlds/tree/ign-garden)
- [bluerov2_ignition](https://github.com/Rezenders/bluerov2_ignition)

## Bluerov Setup

- BlueOS (v1.0.1)

## Installation

### Install ignition

It is necessary to build Ignition from source since we require this [patch](https://github.com/gazebosim/gz-sim/pull/1402), which is not included in any of the available releases yet.

For this, you can follow the instructions in the [ignition documentation](https://gazebosim.org/docs/garden/install_ubuntu_src).

**IMPORTANT**
Instead of using the garden collection

```Bash
$ wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-garden.yaml
$ vcs import < collection-garden.yaml
```

Use this one:

```Bash
$ wget https://raw.githubusercontent.com/clydemcqueen/bluerov2_ignition/clyde_docker/garden.repos
$ vcs import < garden.repos
```

### Install ardusub

Instructions can be found [here](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

The only difference is that is recommend to check out the ArduSub branch

```Bash
$ git clone https://github.com/ArduPilot/ardupilot.git -b Sub-4.1 --recurse
```

```Bash
$ cd ardupilot
$ Tools/environment_install/install-prereqs-ubuntu.sh -y
$ . ~/.profile
```

If you want to use MAC, follow [this instruction](https://ardupilot.org/dev/docs/building-setup-mac.html)

### Install ardusub_plugin

Install ardupilot_gazebo plugin following the instructions in the [repo](https://github.com/ArduPilot/ardupilot_gazebo/tree/ignition-garden)

### Install hackathon workspace

Create new workspace:
```Bash
$ mkdir -p ~/tudelft_hackathon_ws/src
$ cd ~/tudelft_hackathon_ws
```

Clone repos:
```Bash
$ wget https://raw.githubusercontent.com/remaro-network/tudelft_hackathon/ros2/hackathon.rosinstall
$ vcs import src < hackathon.rosintall --recursive
```

Add this to your .bashrc
```Bash
export IGN_GAZEBO_RESOURCE_PATH=$HOME/tudelft_hackathon_ws/src/bluerov2_ignition/models:$HOME/tudelft_hackathon_ws/src/bluerov2_ignition/worlds

export IGN_GAZEBO_RESOURCE_PATH=$HOME/tudelft_hackathon_ws/src/remaro_worlds/models:$HOME/tudelft_hackathon_ws/src/remaro_worlds/worlds:${IGN_GAZEBO_RESOURCE_PATH}
```

Install deps:
```Bash
$ source /opt/ros/foxy/setup.bash
$ rosdep install --from-paths src --ignore-src -r -y
```

Build project:
```Bash
$ cd ~/tudelft_hackathon_ws/
$ colcon build --symlink-install
```

## Run it


ArduSub SITL:
```Bash
$ sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --out=udp:0.0.0.0:14550 --console
```

```Bash
$ ros2 launch tudelft_hackathon bluerov.launch.py simulation:=true
```

## Additional info

### SSH into bluerov (blueOS)

Password: raspberry
```Bash
$ ssh pi@192.168.2.2
```

###  Set SYSID_MYGCS

Ardupilot only allows override msgs from the GCS. It is necessary to change
the [SYSID_MYGCS](https://ardupilot.org/copter/docs/parameters.html#sysid-mygcs-my-ground-station-number) to allow msgs coming from mavros.

The following command can be used to inspect the current sysid.

```Bash
$ ros2 topic echo /uas1/mavlink_source
```

Or

```Bash
$ ros2 param get mavros system_id
```
https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html


set mavros sysid to 255,240
https://discuss.ardupilot.org/t/sending-commands-via-mavros-setpoint-velocity-cmd-vel-in-guided-mode/51610/4


### useful links
https://github.com/mavlink/mavros/issues/1718

### Building ros2 docker images for armv7

https://answers.ros.org/question/362335/unable-to-build-ros-foxy-for-armv7/

### Build ros2 from source in the raspberry

https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html

**Attention**
in the source list add `deb http://packages.ros.org/ros2/ubuntu bullseye  main`
instead

Use only one processor
```Bash
$ export MAKEFLAGS="-j 1"
```

Known bugs: https://github.com/ros2/rcutils/issues/171

Building
```Bash
$ colcon build --symlink-install --executor sequential --cmake-args "-DCMAKE_SHARED_LINKER_FLAGS='-latomic'" "-DCMAKE_EXE_LINKER_FLAGS='-latomic'"
```

### build ros docker image for raspberry

https://github.com/remaro-network/bluerov_ros_docker

```Bash
$ sudo docker run -it --rm --privileged -v /dev:/dev rezenders/bluerov_ping360:foxy ros2 run ping360_sonar ping360_node --ros-args --param device:=/dev/ttyUSB0 --param fallback_emulated:=False
```

### Map ignition topics to ros2 topics

```
ros2 run ros_ign_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar:=/laser_scan
```
