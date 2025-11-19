ARG BASE_IMAGE=ctumrs/mrs_uav_system2:latest
ARG PPA_VARIANT=unstable

FROM ${BASE_IMAGE}

ARG PPA_VARIANT

RUN apt-get -y update

# workaround interractive prompts during apt installations
RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install keyboard-configuration

# INSTALL the MRS UAV System

RUN apt-get -y install software-properties-common curl bash

RUN curl https://ctu-mrs.github.io/ppa2-${PPA_VARIANT}/add_ppa.sh | bash

RUN apt-get -y install ros-jazzy-realsense2*
RUN apt-get -y install ros-jazzy-mrs-realsense
RUN apt-get -y install ros-jazzy-compressed-image-transport
RUN apt-get -y install ros-jazzy-compressed-depth-image-transport

COPY ros_entrypoint.sh /

CMD ["/ros_entrypoint.sh"]
