#!/bin/bash
set -e

# { CMD=$(cat); } << EOF
# . /opt/ros/humble/setup.sh \
#   && cd $HOME/projects/proto_ros2 \
#   && make build && ros2 run proto_ros2 oak_stereo_imu_node.py
# EOF
# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "$CMD" C-m C-m

{ CMD=$(cat); } << EOF
. /opt/ros/humble/setup.sh \
  && cd /home/docker/proto_ros2 \
  && make sim_sandbox
EOF
tmux send-keys -t docker -R C-l C-m
tmux send-keys -t docker -R "$CMD" C-m C-m

# { CMD=$(cat); } << EOF
# . /opt/ros/humble/setup.sh \
#   && cd /home/docker/proto_ros2 \
#   && make build_docker
# EOF
# tmux send-keys -t docker -R C-l C-m
# tmux send-keys -t docker -R "$CMD" C-m C-m

# python3 src/oak_stereo_imu_node.py
