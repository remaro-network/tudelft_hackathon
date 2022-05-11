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

## Installation

Install ardupilot_gazebo plugin following the instructions in the [repo](https://github.com/ArduPilot/ardupilot_gazebo/tree/aaffdc02580980a17f7717e32e520747051811f3)

Create new workspace:
```Bash
$ mkdir -p ~/tudelft_hackathon_ws/src
$ cd ~/tudelft_hackathon_ws/src
```

Clone repos (I will add a .rosintall for this):
```Bash
$ git clone https://github.com/remaro-network/remaro_worlds.git
$ git clone https://github.com/Rezenders/bluerov2_ignition.git
$ git clone https://github.com/remaro-network/mavros_wrapper.git
$ git clone https://github.com/remaro-network/tudelft_hackathon.git
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
