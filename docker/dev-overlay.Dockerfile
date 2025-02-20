FROM nvidia-ros-noetic:latest
SHELL [ "/bin/bash", "-c" ]

# Set env variables for rosdep
ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=noetic

# Create new user with full permission to the volume mount
RUN useradd --create-home appuser

# Copy the src folder for volume mount
USER appuser

WORKDIR /appuser

RUN mkdir -p catkin_ws/src

COPY ./src catkin_ws/src

# Run rosdep to install dependencies
USER root

WORKDIR /appuser/catkin_ws

RUN apt-get update --fix-missing \
  && rosdep update

RUN rosdep install --from-paths src --ignore-src -r -y

# Download Gazebo models
USER appuser

WORKDIR /home/appuser

RUN git clone https://github.com/osrf/gazebo_models.git

RUN mkdir -p .gazebo/models \
  && cp -r gazebo_models/* .gazebo/models \
  && cp -r /appuser/catkin_ws/src/me5413_world/models/* .gazebo/models


# Put your code here to run additional commands
#######################################################################




#######################################################################


# Build the entire package
USER appuser

WORKDIR /appuser/catkin_ws

RUN source /opt/ros/noetic/setup.bash \
  && catkin config --install \
  && catkin build

RUN echo "source /appuser/catkin_ws/devel/setup.bash" >> ~/.bashrc

