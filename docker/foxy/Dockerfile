FROM ros:foxy
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    locales \
    libudev-dev \
    libxml2-dev \
    ros-foxy-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /ros_ws/src && \
  cd /ros_ws/src && \
  git clone https://github.com/jeguzzi/ros-aseba.git --branch ros2 --recurse-submodules && \
  cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install && \
  rm -r build log src

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/ros_ws\/install\/setup.bash"' \
  /ros_entrypoint.sh
