#!/bin/bash
set -e
COLCON_WS=~/colcon_ws

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd $COLCON_WS  \
#   && colcon build --packages-select proto_ros2 \
#   && colcon build --packages-select okvis \
#     --cmake-args -DCMAKE_BUILD_TYPE=Release \
#     -DUSE_NN=OFF \
#     -DBUILD_SUPEREIGHT2_APP=OFF \
#     -DBUILD_ROS2=ON \
#   && source install/setup.bash
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd $COLCON_WS \
#   && colcon build \
#   && source install/setup.bash \
#   && ros2 launch proto_ros2 proto_ros2.launch.py
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf ~/calib_gimbal_data \
#   && cd $COLCON_WS \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_gimbal_inspect
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf ~/calib_gimbal_data \
#   && cd $COLCON_WS \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_camimu_record
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd $COLCON_WS \
#   && colcon build --packages-select proto_ros2 \
#   && colcon build --packages-select ros2_vicon \
#   && source install/setup.bash \
#   && ros2 launch proto_ros2 vicon_experiments.py
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd $COLCON_WS \
#   && colcon build --packages-select proto_ros2 \
#   && colcon build --packages-select ros2_vicon \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 bag2euroc
# " C-m C-m

tmux send-keys -t dev -R C-l C-m
tmux send-keys -t dev -R "\
cd $COLCON_WS \
  && colcon build --packages-select proto_ros2 \
  && source install/setup.bash \
  && ros2 run proto_ros2 rs_multicam_node
" C-m C-m

# python3 scripts/okvis_calib_converter.py
# python3 scripts/plot_poses.py --data_file=/data/gimbal_experiments/odom_fixed/okvis/data.csv
# python3 scripts/plot_poses.py \
#   --data_files \
#   "/data/gimbal_experiments/odom_fixed/okvis/data.csv" \
#   "/data/gimbal_experiments/odom_gimbal_3s/okvis/data.csv" \
#   "/data/gimbal_experiments/odom_gimbal_2s/okvis/data.csv" \
#   --labels \
#   "Fixed", \
#   "Gimbal-3seconds" \
#   "Gimbal-2seconds"

# make build
# python3 scripts/plot_poses.py
# python3 scripts/plot_poses.py \
#   --data_files \
#     /data/gimbal_experiments/run3-active/okvis/data.csv \
#     /data/gimbal_experiments/run4-static/okvis/data.csv \
#   --labels active static
