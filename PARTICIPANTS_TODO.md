# Activities to be done before the hackathon:

#### Follow basic ROS2 tutorials:
* [Beginner: CLI Tools](https://docs.ros.org/en/foxy/Tutorials.html#beginner-cli-tools)
  - [Configuring your ROS 2 environment](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)
  - [Understanding ROS 2 nodes](https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Nodes.html)
  - [Understanding ROS 2 topics](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html)
  - [Understanding ROS 2 services](https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html)
  - [Understanding ROS 2 parameters](https://docs.ros.org/en/foxy/Tutorials/Parameters/Understanding-ROS2-Parameters.html)
  - [Introducing ROS 2 launch](https://docs.ros.org/en/foxy/Tutorials/Launch/CLI-Intro.html)
  - [Recording and playing back data](https://docs.ros.org/en/foxy/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)
* [Beginner: Client Libraries](https://docs.ros.org/en/foxy/Tutorials.html#beginner-client-libraries)
  - [Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
  - [Creating your first ROS 2 package](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html)
  - [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
  - [Writing a simple service and client (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html)

#### Follow basic Ignition tutorials
* [Setting-up a Robot Simulation (Ignition Gazebo)](https://docs.ros.org/en/foxy/Tutorials/Simulators/Ignition/Setting-up-a-Robot-Simulation-Ignition.html)

#### Install ROS2 workspace for the hackathon locally, or run the docker image beforehand to make sure it is working

* [Install locally instructions](https://github.com/remaro-network/tudelft_hackathon#installation)
* [Docker instructions](https://github.com/remaro-network/tudelft_hackathon#run-it-with-docker) (**TODO**)

#### Install Ignition locally, or run the Ignition docker image beforehand to make sure it is working

* [Install locally instructions](https://github.com/remaro-network/tudelft_hackathon#installation)
* [Docker instructions](https://github.com/remaro-network/tudelft_hackathon#run-it-with-docker) (**TODO**)

#### For the more interested participants, reproduce everything and suggest how to improve the training

* For this, it is strongly recommend to do so by opening issues and pull requests
* There are some issues already open. That is a good starting point

# Activities to be done during hackathon:

#### Reproduce everything

* Reproduce everything both in the real bluerov and in Ignition

#### Improve the system
Suggestions:
* Better avoidance algorithm
* Adjust velocity parameters and sonar "field-of-view"
* Test sonar data with SLAM algorithm (#4)
* Add visual to bluerov in ignition (#8)
* Prevent the bluerov to go over the walls (#12)
* Change Ignition initial world camera position (#13)
* Add model for bluerov2 heavy kit version (#14)
* More elaborated world
* Improve formatting
* Improve documentation
* Add tests
