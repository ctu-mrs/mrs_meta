ARG PPA_VARIANT=unstable
ARG BASE_IMAGE=ctumrs/mrs_uav_system_core:${PPA_VARIANT}

FROM ${BASE_IMAGE}

# INSTALL the MRS UAV System

COPY ros_entrypoint.sh /

RUN apt-get update && apt-get -y install ros-${ROS_DISTRO}-realsense2* \
    ros-${ROS_DISTRO}-mrs-realsense ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-compressed-depth-image-transport && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

CMD ["/ros_entrypoint.sh"]
