# ROS2 foxy + hackathon workspace image

```Bash
$ cd tudelft_hackathon_ws/src/tudelft_hackathon
$ sudo docker build -f dockerfiles/ros-foxy-hackathon-dev/Dockerfile -t ros-foxy-hackathon:dev .
```

```Bash
$ sudo docker run -it --rm ros-foxy-hackathon:dev ros2 launch tudelft_hackathon bluerov_bringup_no_ign.launch.py
```
