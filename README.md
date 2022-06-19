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

## Summary
- [Computer setup](https://github.com/remaro-network/tudelft_hackathon#computer-setup)
- [Bluerov setup](https://github.com/remaro-network/tudelft_hackathon#bluerov-setup)
- [Install prerequisites to run with docker](https://github.com/remaro-network/tudelft_hackathon#install-prerequisites)
- [Install locally](https://github.com/remaro-network/tudelft_hackathon#install-locally)
- [Run with docker](https://github.com/remaro-network/tudelft_hackathon#run-it-with-docker)
- [Run with docker with VSCode ](https://github.com/remaro-network/tudelft_hackathon#run-it-with-docker)
- [Run locally](https://github.com/remaro-network/tudelft_hackathon#run-it-locally)
- [Explanation](https://github.com/remaro-network/tudelft_hackathon#explanation)
- [Additional information](https://github.com/remaro-network/tudelft_hackathon#additional-information)
- [Acknowledgments](https://github.com/remaro-network/tudelft_hackathon#acknowledgments)

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

There are 3 options to use this repo.
Install everything locally in your computer. This will require some effort, but when done should be easier to use.
Run everything with docker (read disclaimer bellow):
  - Via the CLI. This option should be faster to reproduce everything, but is a little bit annoying for development.
  - Using VSCode. Requires installing VSCode. Should be easier way to use everything. (work in progress)

**Disclaimer**
Running docker images with graphical user interface is a little bit trick and might not work in all systems.
We tested in a system with ubuntu 20.04 and with a NVIDIA gpu.
It might not work on systems with AMD gpus, and on MAC.
We are going to try to fix this problem until the hackathon, but it is not guaranteed.
In those cases go for the [local installation](https://github.com/remaro-network/tudelft_hackathon#install-locally).

### Install prerequisites to run with docker

- Install docker on your machine. You can find instructions [here](https://docs.docker.com/engine/install/ubuntu/)
- Allow non-root users to manage docker. Instructions [here](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
- Install VSCode. Instructions [here](https://code.visualstudio.com/download)
- Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

### Install locally
#### Install ignition

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

#### Install ardusub

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

#### Install ardusub_plugin

Install ardupilot_gazebo plugin following the instructions in the [repo](https://github.com/ArduPilot/ardupilot_gazebo/tree/ignition-garden)

#### Install hackathon workspace

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

## Run it with docker via CLI

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

### Development with docker via cli

To add your modifications into the docker images you need to rebuild the relevant docker images.
In this case, run the `build-dev-images.sh` script to rebuild them. And make sure to substitute in the `docker run` commands the images from rezenders to you local images. I.e: `rezenders/ignition:hackathon` -> `ignition:hackathon-dev` and  `rezenders/ros-foxy-hackathon` -> `ros-foxy-hackathon:dev`

## Run it with docker with VSCode

Check instructions [here](https://github.com/remaro-network/tudelft_hackathon/blob/ros2/dockerfiles/ros1-2-ignition/README.md)

## Run it locally

### Simulation
ArduSub SITL:
```Bash
$ sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --out=udp:0.0.0.0:14550 --console
```

```Bash
$ ros2 launch tudelft_hackathon bluerov_bringup.launch.py simulation:=true
```

### Bluerov2

```Bash
$ ros2 launch tudelft_hackathon bluerov_bringup.launch.py simulation:=false
```

## Explanation

**WORK IN PROGRESS**

## Additional info

### SSH into bluerov (blueOS)

Password: raspberry
```Bash
$ ssh pi@192.168.2.2
```

## Acknowledgments
This project has received funding from the European Union’s Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.

Pleave visit [our website](https://remaro.eu/) for more info on our project.

![REMARO Logo](https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png)
