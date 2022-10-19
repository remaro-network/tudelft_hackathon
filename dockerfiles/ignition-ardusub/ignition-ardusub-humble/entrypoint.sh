#!/bin/bash

set -e

source "/opt/ros/humble/setup.bash"
source "/home/docker/ardupilot/Tools/completion/completion.bash"
. ~/.profile

exec "$@"
