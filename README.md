# REMARO Summer School Delft 2022 - Underwater robotics hackathon

The overall goal of this hackathon is to provide hands-on training for the Early Stage Researchers (ESRs)
of the REMARO network (but not limited to) on how to architect, program, simulate and implement basic
underwater robot functionalities.

More specifically, the goals for ESRs with background in:
* **Mathematics/Computer Science:** Learn basic robotics workflow and commonly used tools
* **Robotics:** learn cutting-edge tools, and details on how to develop robot-specific system

Thus, this training will address how to setup an underwater robot, such as the [BlueROV2](https://bluerobotics.com/store/rov/bluerov2/),
to run with ROS2, and how to simulate it with Gazebo (Ignition). The idea is that participants
work with a simulation first and then test what is developed in a real BlueROV2, with this they
can begin to understand the challenges and differences of deploying a robot in simulation and in the real world.

The use case selected for this training is “wall avoidance”. Basically, the goal is for the robot
to navigate an environment and not crash into walls using only sonar data. An initial code for a random
avoidance mission is provided, and the idea is that participants work to develop better missions during this training and improve the system in general.

More details on instructions for participants can be found in the [PARTICIPANTS_TODO](https://github.com/remaro-network/tudelft_hackathon/blob/ros2/PARTICIPANTS_TODO.md)

You can check the random wall avoidance behavior on the video:

[![Youtube video](https://user-images.githubusercontent.com/20564040/175087210-6706607d-c2db-4b25-888e-1973e3d093fb.png)](https://www.youtube.com/watch?v=Zv-int_BIJw)

You can find some slides with useful information here (**ADD SLIDES LINK**)

You can find a system architecture of the system developed here (**ADD ARCHITECTURE LINK**)

## Summary
- [Computer setup](https://github.com/remaro-network/tudelft_hackathon#setup)
- [Bluerov setup](https://github.com/remaro-network/tudelft_hackathon#bluerov-setup)
* [Installation](https://github.com/remaro-network/tudelft_hackathon#installation)
  - [Install prerequisites to run with docker](https://github.com/remaro-network/tudelft_hackathon#install-prerequisites-to-run-with-docker)
  - [Install locally](https://github.com/remaro-network/tudelft_hackathon#install-locally)
- [Run it with docker via CLI](https://github.com/remaro-network/tudelft_hackathon#run-it-with-docker-via-cli)
- [Run with docker with VSCode ](https://github.com/remaro-network/tudelft_hackathon#run-it-with-docker-with-vscode)
- [Run locally](https://github.com/remaro-network/tudelft_hackathon#run-it-locally)
- [Explanation](https://github.com/remaro-network/tudelft_hackathon#explanation)
- [Exercises](https://github.com/remaro-network/tudelft_hackathon#exercises)
- [Additional information](https://github.com/remaro-network/tudelft_hackathon#additional-info)
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
- Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)(only needed if you have a nvidia GPU)

### Configure computer to connect with the BlueROV2
Follow [Bluerobotics instructions](https://bluerobotics.com/learn/bluerov2-software-setup/#software-introduction)

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
$ git clone https://github.com/ArduPilot/ardupilot.git -b Sub-4.1 --recurse-submodules
```

```Bash
$ cd ardupilot
$ Tools/environment_install/install-prereqs-ubuntu.sh -y
$ . ~/.profile
```

If you want to use MAC, follow [this instruction](https://ardupilot.org/dev/docs/building-setup-mac.html)

#### Install ardusub_plugin

**IMPORTANT:** Clone into the ignition-garden instead of fortress:

```Bash
$ git clone https://github.com/ArduPilot/ardupilot_gazebo -b ignition-garden
```

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

Create docker network:

```Bash
$ sudo docker network create ros_net
```

### Run Ignition simulation + ardupilot SITL:

If you a NVIDIA GPU:
```Bash
$ xhost +local:root
$ sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all rezenders/ignition:hackathon ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py ardusub:=true mavros_url:='bluerov:14551'
```

If you have an AMD GPU:
```Bash
$ xhost +local:root
$ sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri --group-add video  rezenders/ignition:hackathon ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py ardusub:=true mavros_url:='bluerov:14551'
```

If you have an Intel GPU:
```Bash
$ xhost +local:root
$ sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri:/dev/dri  rezenders/ignition:hackathon ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py ardusub:=true mavros_url:='bluerov:14551'
```

### Run bluerov software:

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

```Bash
$ ros2 launch tudelft_hackathon bluerov_bringup.launch.py simulation:=true ardusub:=true
```

### Bluerov2

```Bash
$ ros2 launch tudelft_hackathon bluerov_bringup.launch.py simulation:=false
```

## Explanation

Simplified system architecture:

![System architecture](https://user-images.githubusercontent.com/20564040/174649275-41a2d0bd-54ed-485f-bfcb-36ffe94dd11c.png)

The system was designed to be used both with a real or simulated BlueROV2. When
used with a simulation, the left nodes are deployed. And when used with the real
robot the right nodes are deployed.The agent and MAVROS nodes are always deployed.

First, let's take a look on how the real BlueROV2 works, then we see how the simulation
is setup to mimic it.

### Real BlueROV2

**BlueROV2:** The BlueROV2 that is going to be used for the hackathon has the [heavy configuration](https://bluerobotics.com/store/rov/bluerov2-upgrade-kits/brov2-heavy-retrofit/). It is equipped with the [Ping 360](https://bluerobotics.com/store/sensors-sonars-cameras/sonar/ping360-sonar-r1-rp/) mechanical scanning sonar mounted in the [standard location](https://bluerobotics.com/learn/ping360-installation-guide-for-the-bluerov2/#mounting-the-ping360-to-the-embluerov2em-frame-heavy). And ArduSub runs on a [Pixhawk](https://bluerobotics.com/store/comm-control-power/control/pixhawk-r1-rp/).

**Ping360 sonar driver:** To get data from the sonar the [ping360_sonar](https://github.com/CentraleNantesRobotics/ping360_sonar) ROS2 package is being used. It provides data in the following topics:

| Topic         | Message type  | Description   |
| ------------- | ------------- | ------------- |
| /scan_image   | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)  | The generated sonar image in gray level|
| /echo  | [ping360_sonar_msg/SonarEcho](https://github.com/CentraleNantesRobotics/ping360_sonar/blob/ros2/ping360_sonar_msgs/msg/SonarEcho.msg)  | Raw sonar data|
| /scan  | [sensor_msgs/LaserScan](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/LaserScan.msg)  | Publishes a LaserScan msg with ranges detected above a certain intensity threshold (0-255). The intensities of the message are scaled down to (0,1)|

Using this package, the Ping360 sonar takes between 6 and 10 seconds to complete a 360 degree rotation.

**ArduSub:** According to their [website](https://www.ardusub.com/) ArduSub is "is a fully-featured open-source solution for remotely operated underwater vehicles (ROVs) and autonomous underwater vehicles (AUVs). ArduSub is a part of the [ArduPilot](https://ardupilot.org/) project, and was originally derived from the ArduCopter code. ArduSub has extensive capabilities out of the box including feedback stability control, depth and heading hold, and autonomous navigation".

With the current configuration, BlueROV2 is not able to calculate its 3D position as there are no sensors that provide enough information for this, such as DVL, underwater GPS, etc.
Thus, it is only possible to operate the robot using [flight modes](https://ardupilot.org/copter/docs/flight-modes.html#gps-dependency) that don't require positioning data, such as MANUAL, STABILIZE, and DEPTH HOLD. See [ArduSub flight modes](https://www.ardusub.com/reference/ardusub/features-while-in-operation.html#flight-modes).
Consequently, it is not possible to send waypoints or velocity commands to the robot, the only way to operate it is by overriding the [RC inputs](https://www.ardusub.com/developers/rc-input-and-output.html#rc-inputs).

**MAVROS:** The [MAVROS](https://github.com/mavlink/mavros) package is used to bridge the communication between ROS and mavlink, which is the communication protocol used by ArduSub. Which means MAVROS can be used as a bridge between ROS and ArduSub. To connect it to ArduSub it necesary to set the following parameters:
- `fcu_url`: Address of the flight controller unit. For this case: `udp://192.168.2.1:14550@192.168.2.2:14555`

MAVROS provides an extensive number of topics and services to interact with autopilots, you can check the full list [here](http://wiki.ros.org/mavros). For our use case, the topics and services being used are:

| Topic         | Message type      | Description   |
| ------------- | -------------     | ------------- |
| mavros/state   | [mavros_msgs/State](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/State.msg) |Provides the FCU state |
| mavros/rc/override  | [mavros_msgs/OverrideRCIn](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/OverrideRCIn.msg) | Send RC override message to FCU |

| Service         | Message type      | Description   |
| ------------- | -------------     | ------------- |
| mavros/set_mode |[mavros_msgs/SetMode](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/srv/SetMode.srv)| Set FCU flight mode |
| mavros/cmd/arming |[mavros_msgs/CommandBool](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/srv/CommandBool.srv)| Change arming status|


To simplify interactions with MAVROS, a [wrapper](https://github.com/remaro-network/mavros_wrapper) was setup with the basic functionalities needed for this hackathon. It works like this:

```Python
from mavros_wrapper.ardusub_wrapper import * # Import wrapper

ardusub = BlueROVArduSubWrapper("ardusub_node") # create new instance with desired node name
status = ardusub.status # to get FCU status
ardusub.set_mode("MANUAL") # set FCU flight mode to MANUAL
ardusub.arm_motors(True) # arm motors
ardusub.toogle_rc_override(True) # start overriding RC
ardusub.set_rc_override_channels(forward=0.5) # set values to override
```

**Agent:** The agent node is the one that decides how the robot should behave.
For this hackathon, we setup two different nodes with simple behaviors:

The [bluerov_agent](https://github.com/remaro-network/tudelft_hackathon/blob/ros2/scripts/bluerov_agent.py) has the following behavior. The robot set its flight mode to `MANUAL`, then arms the thrusters, then go around in a square path once. For this, the agent uses the topics & services listed in the mavros section.

Let's break the code and understand what's going on below the surface.

The main function:
```Python
if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    ardusub = BlueROVArduSubWrapper("ardusub_node")

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    mission(ardusub)
```
First the `rclpy` library is initialized. Then the ardusub node is created using the `BlueROVArduSubWrapper` class defined in the `mavros_wrapper` package. Following, a new thread is created to spin the ardusub node in parallel, this is necessary because we are going to use some sleep functions in the main thread later and we want the ardusub node to keep spinning despite of that. Lastly, the `mission` function is called, which is where the mission of the robot is defined.

Now let's take a look in the `mission` function
```Python
def mission(ardusub):

    service_timer = ardusub.create_rate(2)
    while ardusub.status.mode != "MANUAL":
        ardusub.set_mode("MANUAL")
        service_timer.sleep()

    print("Manual mode selected")

    while ardusub.status.armed == False:
        ardusub.arm_motors(True)
        service_timer.sleep()

    print("Thrusters armed")

    print("Initializing mission")

    timer = ardusub.create_rate(0.5) # Hz

    ardusub.toogle_rc_override(True)
    ardusub.set_rc_override_channels(forward=0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(forward=-0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=-0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=0)
    ardusub.toogle_rc_override(False)

    print("Mission completed")
```
A `service_timer` timer is created with 2Hz rate. Then we have a while loop that keeps trying to change the flight mode to `MANUAL` until the flight mode is set to `MANUAL`, note that the `ardusub.set_mode("MANUAL")` method from the mavros_wrapper explained earlier is used for this, and that the `service_timer` is used to make the loop wait for 0.5 seconds before repeating. Following, we have a similar loop to arm the motors.

After that, we create a new timer with 0.5Hz rate. The `ardusub.toogle_rc_override(True)` method is called to make the ardusub node start publishing messages in the `/mavros/rc/override` topic. Then, we use the method `ardusub.set_rc_override_channels(forward=0.5)` to make the robot move forward with half of its thrust, this is achieved internally by the ardusub node by publishing the appropriate message in the `/mavros/rc/override` topic. Following, the program sleeps for 2 seconds and then we make the robot move again to other directions, in order to go "around" a square. And that is it.


The [random_wall_avoidance](https://github.com/remaro-network/tudelft_hackathon/blob/ros2/scripts/random_wall_avoidance.py) has the following behavior. The robot set its flight mode to `ALT HOLD`, then arms the thrusters, then starts moving forward. The agent subscribes to the sonar topic and every time it receives sonar readings it checks if there is an obstacle in its front, e.g. 180°, closer than a certain threshold, e.g. 1.25m, and in case there is it rotates randomly until there are no more obstacle. For this, the agent uses the topics & services listed in the mavros section, and the `/scan` topic described in the ping360 driver section.

### Simulated BlueROV2

**Simulated BlueROV2:** To simulate the BlueROV2 we are using Gazebo (Ignition). Unfortunately, until the moment of writing this readme, there is no sonar plugin for Ignition. Thus, we are using a lidar plugin instead, configured to have the same speed and measurement range as the Ping360 sonar. The bluerov2 model is [here](https://github.com/Rezenders/bluerov2_ignition/blob/main/models/bluerov2/model.sdf) and the bluerov2 model with a lidar is [here](https://github.com/Rezenders/bluerov2_ignition/blob/main/models/bluerov2_lidar/model.sdf). The world being used for the simulation can be found [here](https://github.com/remaro-network/remaro_worlds/blob/ign-garden/worlds/room_walls.world), note that there is a buoyancy plugin that sets the "water" density to 1000kg/m3.

**Simulated "sonar" bridge:**  In order to have access to the simulated lidar data with ROS2 we run a bridge between ignition transport and ROS2. Something like this:

Via command line:
```
ros2 run ros_ign_bridge parameter_bridge lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan -r /lidar:=/scan
```

With launch file:
```
package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=['lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
    remappings=[('/lidar','/scan')],
    output='screen'
```

Note that the topic where the lidar data is published has the same name (`/scan`) and type (`sensor_msgs/LaserScan`) as the topic published by the ping360 driver.

**ArduSub Sofware In The Loop (SITL):** Since when running the simulation we don't have a board with an autopilot installed, we simulate Ardusub as a SITL.

**Ardupilot gazebo plugin:** Bridge between Ignition and Ardusub. More info can be found [here](https://gazebosim.org/api/gazebo/7.0/ardupilot.html).

**MAVROS:** The only difference is that for the simulation we need to use a different `fcu_url`. In this case, `udp://:14551@:14555`.

**Agent:** Since all the interfaces are the same, the agent nodes are the same for both simulation and the real robot.

## Exercises

The main exercise of this training is to implement an avoidance algorithm based on potential fields, you can find more info on the issue [#36](https://github.com/remaro-network/tudelft_hackathon/issues/36).

Check [PARTICIPANTS_TODO](https://github.com/remaro-network/tudelft_hackathon/blob/ros2/PARTICIPANTS_TODO.md#activities-to-be-done-during-hackathon) for extra info.

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
