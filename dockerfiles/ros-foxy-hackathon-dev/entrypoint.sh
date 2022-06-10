#!/bin/bash

set -e

source "/opt/ros/foxy/setup.bash"
source "/tudelft_hackathon_ws/install/setup.bash"

exec "$@"
