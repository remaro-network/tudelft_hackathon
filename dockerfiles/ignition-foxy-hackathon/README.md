# Ignition bluerov2 image

```Bash
$ sudo docker build -t ignition:foxy-hackathon .
```

```Bash
$ xhost +local:root
$ sudo docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all ignition:foxy-hackathon ign gazebo -v3 -r underwater.world
```
