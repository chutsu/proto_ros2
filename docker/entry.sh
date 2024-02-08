#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

# Start Micro-XRCE Agent
MicroXRCEAgent udp4 -p 8888 &

# Start an interactive shell
/bin/bash
cd $HOME
