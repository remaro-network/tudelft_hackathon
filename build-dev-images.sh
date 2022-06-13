#!/bin/bash

sudo docker build -f dockerfiles/ignition-foxy-hackathon-dev/Dockerfile -t ignition:hackathon-dev .
sudo docker build -f dockerfiles/ros-foxy-hackathon-dev/Dockerfile -t ros-foxy-hackathon:dev .
