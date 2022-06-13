#!/bin/bash

# ignition:foxy
sudo docker build -t rezenders/ignition:foxy -f ignition/Dockerfile ignition/
sudo docker push rezenders/ignition:foxy

# ignition:foxy-nvidia
sudo docker build -t rezenders/ignition:foxy-nvidia -f ignition-nvidia/Dockerfile ignition-nvidia/
sudo docker push rezenders/ignition:foxy-nvidia

# ignition:foxy-nvidia-ardusub
sudo docker build -t rezenders/ignition:foxy-nvidia-ardusub -f ignition-foxy-ardusub/Dockerfile ignition-foxy-ardusub/
sudo docker push rezenders/ignition:foxy-nvidia-ardusub

# ignition:hackathon
sudo docker build --no-cache  -t rezenders/ignition:hackathon -f ignition-foxy-hackathon/Dockerfile ignition-foxy-hackathon/
sudo docker push rezenders/ignition:hackathon

# ignition:hackathon
sudo docker build --no-cache  -t rezenders/ros-foxy-hackathon -f ros-foxy-hackathon/Dockerfile ros-foxy-hackathon/
sudo docker push rezenders/ros-foxy-hackathon
