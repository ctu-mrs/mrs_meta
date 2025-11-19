#!/bin/bash

. /opt/ros/jazzy/setup.sh

#. /home/ubuntu/ws/install/setup.sh

if [-z UAV_NAME ]; then
  echo "$0: the UAV_NAME environment variable is not set"
  exit 1
fi

#cd /home/ubuntu/ws
ros2 launch mrs_realsense uav.launch.py $@

# docker run --privileged -it -v ./custom_config.yaml:/home/ubuntu/ws/custom_config.yaml mrs_realsense:latest custom_config:=custom_config.yaml
