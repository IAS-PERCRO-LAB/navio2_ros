#!/bin/bash

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
source "/home/docker/ws/install/local_setup.bash" --
exec "$@"
