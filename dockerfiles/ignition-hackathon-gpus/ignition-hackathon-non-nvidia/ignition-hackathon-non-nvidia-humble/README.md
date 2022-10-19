#Ignition-non-nvidia image

## Info
http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration

## Build image

```Bash
$ sudo docker build -t ignition:foxy-non-nvidia .
```

## AMD

```Bash
$ xhost +local:root
$ sudo docker run --rm -it --device=/dev/dri --group-add video --volume=/tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" ignition:non-nvidia ign gazebo
```

## Intel

```Bash
$ xhost +local:root
$ sudo docker run --rm -it --device=/dev/dri:/dev/dri --volume=/tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" ignition:non-nvidia ign gazebo
```
