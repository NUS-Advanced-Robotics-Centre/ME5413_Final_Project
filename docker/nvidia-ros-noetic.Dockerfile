# Ubuntu 20.04 base image with NVIDIA CUDA and ROS Noetic
FROM nvidia/cudagl:11.2.1-base-ubuntu20.04
SHELL ["/bin/bash", "-c"]

# Install apt packages
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  locales \
  lsb-release

RUN dpkg-reconfigure locales

RUN apt-get update && apt-get install -y \
  git \
  g++ \
  cmake \
  gnupg \
  gnupg1 \
  gnupg2 \
  wget

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full

RUN apt-get install -y --no-install-recommends python3-rosdep

RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install python3 dependencies
RUN apt-get install -y \
  python3-pip \
  python3-catkin-tools \
  python3-opencv

COPY ./docker/ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]

