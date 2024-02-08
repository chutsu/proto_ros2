#!/bin/bash
set -e

PREFIX=~/projects
COLCON_WS=~/colcon_ws

install_px4() {
  # Install PX4
  if [ ! -d $PREFIX/PX4-Autopilot ]; then
    cd $PREFIX
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  fi
  cd $PREFIX/PX4-Autopilot
  bash ./Tools/setup/ubuntu.sh
  make px4_sitl
}

install_micro_xrce_dss_agent() {
  # Install Micro-XRCE-DSS-Agent
  if [ ! -d $PREFIX/Micro-XRCE-DDS-Agent ]; then
    cd $PREFIX
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  fi
  cd $PREFIX/Micro-XRCE-DDS-Agent
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
}

install_ros2_pkgs() {
  # Install ROS2 components
  cd $COLCON_WS/src
  if [ ! -d $COLCON_WS/src/px4_msgs ]; then
    git clone https://github.com/PX4/px4_msgs.git
  fi
  if [ ! -d $COLCON_WS/src/px4_ros_com ]; then
    git clone https://github.com/PX4/px4_ros_com.git
  fi

  cd $COLCON_WS
  colcon build --packages-select px4_msgs
  colcon build --packages-select px4_ros_com
}


# install_px4
# install_micro_xrce_dss_agent
install_ros2_pkgs
