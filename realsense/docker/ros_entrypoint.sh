#!/bin/bash

# set ROS_DISTRO if it's missing
if [ -z "$ROS_DISTRO" ]; then
  for dir in /opt/ros/*; do
    if [ -f "$dir/setup.bash" ]; then
      export ROS_DISTRO=$(basename "$dir")
    fi
  done
fi

source /opt/ros/$ROS_DISTRO/setup.bash

if [ -z "$UAV_NAME" ]; then
  echo "$0: the UAV_NAME environment variable is not set"
  exit 1
fi

ros2 launch mrs_realsense uav.launch.py $@

# docker run --privileged -it -v ./custom_config.yaml:/home/ubuntu/ws/custom_config.yaml ctumrs/realsense:latest custom_config:=custom_config.yaml
