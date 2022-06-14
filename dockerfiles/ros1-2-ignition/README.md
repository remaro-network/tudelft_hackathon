# ROS2 Workspace for REMARO's hackaton
This template will get you set up using ROS1 together with ROS2 in VSCode as your IDE.
It is structured as follows:
```
├──ros1-2-ignition
│  ├──setup-ign-ardupilot-sitl.sh
│  ├──ardupilot
|  |  └── ...
│  ├──ardupilot_gazebo
|  |  └── ...
│  ├──ros1_ws                       # ROS Noetic workspace
|  |  ├── src
|  |  |   └── ...
│  ├──ros2_ws                       # ROS Foxy workspace
|  |  ├── src
|  |  |   ├──bluerov2_ignition      # Code for Bluerov2 simulation in Ignition
|  |  |   ├──remaro_worlds          # Ignition simulated environments
|  |  |   └── ...
│  |  └── ...
│  └── ...
└── ...
```

Note that ardupilot, bluerov2_ignition and remaro_worlds are included in this repository as git submodules.

## Some things to keep in mind

### ROS1 bridge for ROS2
This repo is extended to include both ROS1 noetic and ROS2 foxy. The ros1/ workspace is set up to build ROS1 packages with catkin for noetic and the ros2/ workspace is set up to build ROS2 packages with colcon. 

The script [`ros2/ros1_bridge.sh`](ros2/ros1_bridge.sh) starts the dynamic ros1/2 bridge to forward messages between nodes of both ros versions. 

### Shortcuts
ROS need sourcing of its setup files. Since we have both ROS1 and 2, we need to source the ROS versions that we are using (usually ROS2).  Some shortcuts are defined in the .bashrc file for convenience. They are listed here:  
- `sf`: as in source foxy, resolves to 'source /opt/ros/foxy/setup.bash'  
- `sn`: as in source noetic, resolves to 'source /opt/ros/noetic/setup.bash'  
- `s`: as in source (local), resolves to 'source install/setup.bash' to source local environment in ROS2  
- `m`: as in make, resolves to 'bash build.sh', requires a build script to be present in the local directory  

In this hackaton, you will source ROS2 (foxy), and therefore type `sf` and `s`. Note that `s` must be run from `ros2_ws` directory.

Now, let's get hands-on!

# Open the container in vscode
- Clone the hackaton's repo in your computer:
- Go to the folder containing this Dev container (i.e. ros1-2-ignition folder)
- Open it from VSCode (File->Open Folder). 
- click on the little green square in the bottom left corner, which should bring up the container dialog

![template_vscode_bottom](https://user-images.githubusercontent.com/6098197/91332638-5d47b780-e781-11ea-9fb6-4d134dbfc464.png)

- In the dialog, select "Remote Containers: Reopen in container"

VSCode will build the dockerfile inside of `.devcontainer` for you.  If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

![template_container](https://user-images.githubusercontent.com/6098197/91332895-adbf1500-e781-11ea-8afc-7a22a5340d4a.png)

- If you want to work in your docker container from terminator (which I strongly recommend), type `terminator` in the VSCode terminal.

# Setup Ardupilot and ROS in your docker
Lucky for you, we've set up a bash file that does this steps -almost entirely- for you!

- Run the file `setup-ign-ardupilot-sitl.sh` to setup the ignition worlds and ardupilot installation:
    ```
    chmod +x setup-ign-ardupilot-sitl.sh
    ./setup-ign-ardupilot-sitl.sh
    ```
- Build your ROS2 packages:
    ```
    cd ros2_ws
    colcon build
    ```
- Source your ROS2 installation and your local ROS packages by using the shortcuts metioned at the beginning of this README:
    ```
    sf # source foxy
    s  # source your ROS packages
    ```

# Test that everything is working!
Run Ardupilot with software in the loop (SITL):
```
cd ardupilot
Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --out=udp:0.0.0.0:14550 --console
```
Run Ignition with the BlueROV
```
ign gazebo auv_controls.sdf
```