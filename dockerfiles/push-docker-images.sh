#!/bin/bash

# ignition:foxy
docker build -t rezenders/ignition:foxy -f ignition/Dockerfile ignition/
docker push rezenders/ignition:foxy

# ignition:foxy-ardusub
docker build --no-cache -t rezenders/ignition:foxy-ardusub -f ignition-foxy-ardusub/Dockerfile ignition-foxy-ardusub/
docker push rezenders/ignition:foxy-ardusub

# ignition:hackathon
docker build --no-cache  -t rezenders/ignition:hackathon -f ignition-foxy-hackathon/Dockerfile ignition-foxy-hackathon/
docker push rezenders/ignition:hackathon

# ignition:hackathon-nvidia
docker build --no-cache -t rezenders/ignition:hackathon-nvidia -f ignition-nvidia/Dockerfile ignition-nvidia/
docker push rezenders/ignition:hackathon-nvidia

# ignition:hackathon-non-nvidia
docker build --no-cache -t rezenders/ignition:hackathon-non-nvidia -f ignition-non-nvidia/Dockerfile ignition-non-nvidia/
docker push rezenders/ignition:hackathon-non-nvidia

#ros-foxy-hackathon
docker build --no-cache  -t rezenders/ros-foxy-hackathon -f ros-foxy-hackathon/Dockerfile ros-foxy-hackathon/
docker push rezenders/ros-foxy-hackathon
