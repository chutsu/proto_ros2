#!/bin/bash
set -e

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/proto_ws && colcon build && source $HOME/proto_ws/install/setup.bash && ros2 run proto_ros2 sbgc_node
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/proto_ws \
#   && colcon build \
#   && source install/setup.bash \
#   && ros2 launch proto_ros2 proto_ros2.launch.py
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf ~/calib_gimbal_data \
#   && cd ~/proto_ws \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_gimbal_inspect
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf ~/calib_gimbal_data \
#   && cd ~/proto_ws \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_camimu_record
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/proto_ws \
#   && colcon build \
#   && source install/setup.bash  \
#   && ros2 run proto_ros2 okvis_node \
#     --ros-args \
#     -p config_path:=/home/chutsu/config_realsense_D435i_Chris.yaml \
#     -p dbow_dir:=/home/chutsu/projects/okvis2
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   cd ~/proto_ws \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 rs_node
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/proto_ws \
#   && colcon build --packages-select proto_ros2 \
#   && colcon build --packages-select ros2_vicon \
#   && source install/setup.bash \
#   && ros2 launch proto_ros2 vicon_experiments.py
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd ~/proto_ws \
#   && colcon build --packages-select proto_ros2 \
#   && colcon build --packages-select ros2_vicon \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 bag2euroc
# " C-m C-m

# python3 scripts/okvis_calib_converter.py
# python3 scripts/plot_poses.py --data_file=/data/gimbal_experiments/odom_fixed/okvis/data.csv
python3 scripts/plot_poses.py \
  --data_files \
  "/data/gimbal_experiments/odom_fixed/okvis/data.csv" \
  "/data/gimbal_experiments/odom_gimbal_3s/okvis/data.csv" \
  "/data/gimbal_experiments/odom_gimbal_2s/okvis/data.csv" \
  --labels \
  "Fixed", \
  "Gimbal-3seconds" \
  "Gimbal-2seconds"

# make build
