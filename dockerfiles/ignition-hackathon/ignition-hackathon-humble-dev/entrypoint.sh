#!/bin/bash

set -e

source "/opt/ros/humble/setup.bash"
source "/home/docker/tudelft_hackathon_ws/install/setup.bash"
source "/home/docker/ardupilot/Tools/completion/completion.bash"
. ~/.profile

exec "$@"
