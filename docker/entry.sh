#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

# Start Micro-XRCE Agent
nohup MicroXRCEAgent udp4 -p 8888 > agent_log.txt 2>&1 &

# Start an interactive shell
/bin/bash
