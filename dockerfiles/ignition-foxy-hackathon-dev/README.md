# Ignition bluerov2 image

```Bash
$ cd tudelft_hackathon_ws/src/tudelft_hackathon
$ sudo docker build -f dockerfiles/ignition-foxy-hackathon-dev/Dockerfile -t ignition:foxy-hackathon-dev .
```

```Bash
$ xhost +local:root
$ sudo docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all ignition:foxy-hackathon-dev ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py
```
