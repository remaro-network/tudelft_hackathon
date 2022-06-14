# Underwater robotics hackathon

The intended goal of this hackathon is to provide an initial training on how to
setup a underwater robot, such as the [bluerov2](https://bluerobotics.com/store/rov/bluerov2/),
to run with ROS2, and how to simulate it with Ignition.

The idea is to perform a simple mission of avoiding walls using sonar readings.
A initial mission is provided and the goal is to improve this mission throughout
the training.

You can check the intended behavior on the video:

**ADD VIDEO HERE**

You can find some slides with useful information here (**ADD SLIDES LINK**)

You can find a system architecture of the system developed here (**ADD ARCHITECTURE LINK**)

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
$ wget https://raw.githubusercontent.com/Rezenders/bluerov2_ignition/main/garden.repos
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
$ vcs import src < hackathon.rosinstall --recursive
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

## Run it locally

ArduSub SITL:
```Bash
$ sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --out=udp:0.0.0.0:14550 --console
```

```Bash
$ ros2 launch tudelft_hackathon bluerov_bringup.launch.py simulation:=true
```

## Run it with docker

Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

Create docker network:

```Bash
$ sudo docker network create ros_net
```

Run Ignition simulation + ardupilot SITL:

```Bash
$ xhost +local:root
$ sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all rezenders/ignition:hackathon ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py ardusub:=true mavros_url:='bluerov:14551'
```

Run bluerov software:

```Bash
$ sudo docker run -it --rm --name bluerov --net ros_net rezenders/ros-foxy-hackathon ros2 launch tudelft_hackathon bluerov_bringup_no_ign.launch.py fcu_url:=udp://:14551@ignition:14555
```

## Additional info

### SSH into bluerov (blueOS)

Password: raspberry
```Bash
$ ssh pi@192.168.2.2
```

## Acknowledgements
This project has received funding from the European Union’s Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.

Pleave visit [our website](https://remaro.eu/) for more info on our project.

![REMARO Logo](https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png)
