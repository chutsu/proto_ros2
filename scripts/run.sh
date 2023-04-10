#!/bin/bash
set -e

{ CMD=$(cat); } << EOF
. /opt/ros/humble/setup.sh \
  && cd $HOME/projects/proto_ros2 \
  && make sim_sandbox
EOF

tmux send-keys -t dev -R C-l C-m
tmux send-keys -t dev -R "$CMD" C-m C-m
