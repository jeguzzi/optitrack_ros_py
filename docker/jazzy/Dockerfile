FROM ros:jazzy
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    python3-pykdl \
    python3-numpy \
    python3-pip \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install netifaces git+https://github.com/jeguzzi/natnet_py.git --break-system-packages

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

COPY optitrack_ros_py /ros_ws/src/optitrack_ros_py
COPY optitrack_msgs /ros_ws/src/optitrack_msgs

RUN cd /ros_ws \
    && source /ros_entrypoint.sh \
    && colcon build --install-base /opt/ros/ws --merge-install \
    && rm -r /ros_ws

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/opt\/ros\/ws\/setup.bash"' \
  /ros_entrypoint.sh
