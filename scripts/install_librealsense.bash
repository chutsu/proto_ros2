#!/bin/bash
set -e

# Dependencies
sudo apt-get install -qyy \
  libssl-dev \
  libusb-1.0-0-dev \
  libudev-dev \
  pkg-config \
  libgtk-3-dev \
  cmake

# Ubuntu 22.04 specific Dependencies
sudo apt-get install \
  libglfw3-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev at

# Git clone librealsense
cd /usr/local/src
if [ ! -d /usr/local/src/librealsense ]; then
  cd /usr/local/src
  sudo git clone https://github.com/IntelRealSense/librealsense
fi

# Build librealsense
cd /usr/local/src/librealsense
sudo ./scripts/setup_udev_rules.sh
sudo mkdir -p build
cd build
sudo cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false
sudo make
