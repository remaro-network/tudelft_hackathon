# Ignition bluerov2 image

```Bash
$ sudo docker build -t ignition:foxy-hackathon .
```

```Bash
$ xhost +local:root
$ sudo docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all ignition:foxy-hackathon ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py
```
