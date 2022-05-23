# tudelft_hackathon
Repository for the TU Delft hackathon

## Setup

Tested with:
- Ubuntu 20.04
- ROS2 foxy
- Ignition garden (commit 7063d41)
- ArduPilot
- this [version](https://github.com/ArduPilot/ardupilot_gazebo/tree/aaffdc02580980a17f7717e32e520747051811f3) of ardupilot_gazebo plugin
- [mavros2](https://github.com/mavlink/mavros)
- ignition garden version of [remaro_world](https://github.com/remaro-network/remaro_worlds/tree/ign-garden)
- [bluerov2_ignition](https://github.com/Rezenders/bluerov2_ignition)

## Bluerov Setup

- BlueOS (v1.0.1)

## Installation

Install ardupilot_gazebo plugin following the instructions in the [repo](https://github.com/ArduPilot/ardupilot_gazebo/tree/aaffdc02580980a17f7717e32e520747051811f3)

Create new workspace:
```Bash
$ mkdir -p ~/tudelft_hackathon_ws/src
$ cd ~/tudelft_hackathon_ws
```

Clone repos (I will add a .rosintall for this):
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
$ colcon build
```

## Run it

Ignition gazebo:
```Bash
$ ign gazebo -v 3 -r bluerov_pipeline.world
```

ArduSub SITL:
```Bash
$ sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --out=udp:0.0.0.0:14550 --console
```

Bluerov agent:
```Bash
$ ros2 launch tudelft_hackathon bluerov.launch
```

## Additional info

### SSH into bluerov (blueOS)

Password: raspberry
```Bash
$ ssh pi@192.168.2.2
```
### Ping360 address

```
udp://192.168.2.2:9092
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
colcon build --symlink-install --executor sequential --cmake-args "-DCMAKE_SHARED_LINKER_FLAGS='-latomic'" "-DCMAKE_EXE_LINKER_FLAGS='-latomic'"
```

### build ros docker image for raspberry

https://github.com/remaro-network/bluerov_ros_docker

```Bash
sudo docker run -it --rm --privileged -v /dev:/dev rezenders/bluerov_ping360:foxy ros2 run ping360_sonar ping360_node --ros-args --param device:=/dev/ttyUSB0 --param fallback_emulated:=False
```
