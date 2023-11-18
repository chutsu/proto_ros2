#!/bin/bash
set -e

# { CMD=$(cat); } << EOF
# . /opt/ros/humble/setup.sh \
#   && cd $HOME/projects/proto_ros2 \
#   && make build && ros2 run proto_ros2 oak_stereo_imu_node.py
# EOF
# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "$CMD" C-m C-m

# { CMD=$(cat); } << EOF
# . /opt/ros/humble/setup.sh \
#   && cd /home/docker/proto_ros2 \
#   && make sim_sandbox
# EOF
# tmux send-keys -t docker -R C-l C-m
# tmux send-keys -t docker -R "$CMD" C-m C-m

# { CMD=$(cat); } << EOF
# . /opt/ros/humble/setup.sh \
#   && cd /home/docker/proto_ros2 \
#   && make build_docker
# EOF
# tmux send-keys -t docker -R C-l C-m
# tmux send-keys -t docker -R "$CMD" C-m C-m

# rm -rf /home/chutsu/record
# python3 src/oak_stereo_imu_node.py --output_path /home/chutsu/record
# CMD="python3 src/oak_stereo_imu_node.py --output_path /home/chutsu/record"
# # CMD="python3 src/oak_stereo_imu_record.py"
# CMD="python3 src/oak_imu_node.py"
# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "cd ~/projects/proto_ros2 && $CMD" C-m C-m


# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/proto_ws && colcon build && source $HOME/proto_ws/install/setup.bash && ros2 run proto_ros2 sbgc_node
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/proto_ws \
#   && colcon build \
#   && source install/setup.bash  \
#   && ros2 run proto_ros2 rs_node --cmake-args -DCMAKE_BUILD_TYPE=Release
# " C-m C-m

tmux send-keys -t dev -R C-l C-m
tmux send-keys -t dev -R "\
cd ~/proto_ws \
  && colcon build \
  && source install/setup.bash  \
  && ros2 run proto_ros2 okvis_node
" C-m C-m
