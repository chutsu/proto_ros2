#!/bin/bash
set -e

# { CMD=$(cat); } << EOF
# . /opt/ros/humble/setup.sh \
#   && mkdir -p $HOME/proto_ws \
#   && cd $HOME/proto_ws \
#   && colcon build --parallel-workers 4 --packages-select proto_ros2 \
#   && . install/setup.sh \
#   && ros2 launch proto_ros2 calib_sim.launch.py
# EOF

{ CMD=$(cat); } << EOF
. /opt/ros/humble/setup.sh \
  && cd $HOME/projects/proto/proto_ros2 \
  && make gz_gimbal
EOF

# { CMD=$(cat); } << EOF
#   cd $HOME/projects/proto/proto_ros2/gazebo/ \
#   && GZ_SIM_RESOURCE_PATH=$PWD/gazebo/models:$PWD/gazebo/worlds gz sim worlds/calibration.sdf
# EOF

tmux send-keys -t dev -R C-l C-m
tmux send-keys -t dev -R "$CMD" C-m C-m
