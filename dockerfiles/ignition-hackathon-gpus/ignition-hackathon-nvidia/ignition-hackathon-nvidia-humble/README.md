#Ignition-nvidia image

## Install nvidia-docker
Follow instructions on this [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian)

## Build image

```Bash
$ sudo docker build -t ignition:foxy-nvidia .
```

## Run image

```Bash
$ xhost +local:root
$ sudo docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all ignition:nvidia ign gazebo
```
