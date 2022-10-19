#!/bin/bash

# ignition:foxy
#docker build --no-cache -t rezenders/ignition:foxy -f ignition/ignition-foxy/Dockerfile ignition/ignition-foxy/
#docker push rezenders/ignition:foxy

# ignition:humble
docker build -t rezenders/ignition:humble -f ignition/ignition-humble/Dockerfile ignition/ignition-humble/
docker push rezenders/ignition:humble

# ignition:ardusub-foxy
#docker build --no-cache -t rezenders/ignition:ardusub-foxy -f ignition-ardusub/ignition-ardusub-foxy/Dockerfile ignition-ardusub/ignition-ardusub-foxy/
#docker push rezenders/ignition:ardusub-foxy

# ignition:ardusub-humble
docker build -t rezenders/ignition:ardusub-humble -f ignition-ardusub/ignition-ardusub-humble/Dockerfile ignition-ardusub/ignition-ardusub-humble/
docker push rezenders/ignition:ardusub-humble

# ignition:hackathon-foxy
#docker build --no-cache  -t rezenders/ignition:hackathon-foxy -f ignition-hackathon/ignition-hackathon-foxy/Dockerfile ignition-hackathon/ignition-hackathon-foxy/
#docker push rezenders/ignition:hackathon-foxy

# ignition:hackathon-humble
docker build -t rezenders/ignition:hackathon-humble -f ignition-hackathon/ignition-hackathon-humble/Dockerfile ignition-hackathon/ignition-hackathon-humble/
docker push rezenders/ignition:hackathon-humble

# ignition:hackathon-nvidia-foxy
#docker build --no-cache -t rezenders/ignition:hackathon-nvidia -f ignition-nvidia/Dockerfile ignition-nvidia/
#docker push rezenders/ignition:hackathon-nvidia

# ignition:hackathon-nvidia-humble
docker build -t rezenders/ignition:hackathon-nvidia-humble -f ignition-hackathon-gpus/ignition-hackathon-nvidia/ignition-hackathon-nvidia-humble/Dockerfile ignition-hackathon-gpus/ignition-hackathon-nvidia/ignition-hackathon-nvidia-humble/
docker push rezenders/ignition:hackathon-nvidia-humble

# ignition:hackathon-non-nvidia-foxy
#docker build --no-cache -t rezenders/ignition:hackathon-non-nvidia -f ignition-non-nvidia/Dockerfile ignition-non-nvidia/
#docker push rezenders/ignition:hackathon-non-nvidia

# ignition:hackathon-non-nvidia-foxy
docker build -t rezenders/ignition:hackathon-non-nvidia -f ignition-hackathon-gpus/ignition-hackathon-non-nvidia/ignition-hackathon-non-nvidia-humble/Dockerfile ignition-hackathon-gpus/ignition-hackathon-non-nvidia/ignition-hackathon-non-nvidia-humble/
docker push rezenders/ignition:hackathon-non-nvidia-humble

#ros-hackathon-foxy
#docker build --no-cache  -t rezenders/ros-foxy-hackathon -f ros-foxy-hackathon/Dockerfile ros-foxy-hackathon/
#docker push rezenders/ros-foxy-hackathon

#ros-hackathon-humble
docker build -t rezenders/ros-hackathon-humble -f ros-hackathon/ros-hackathon-humble/Dockerfile ros-hackathon/ros-hackathon-humble/
docker push rezenders/ros-hackathon-humble
