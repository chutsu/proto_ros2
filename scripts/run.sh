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
# cd $COLCON_WS \
#   && colcon build \
#   && source install/setup.bash \
#   && ros2 launch proto_ros2 perception_module.launch.py
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf  ~/calib_camera-rs0 \
#   && cd $COLCON_WS \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_camera_record 0 /home/chutsu/calib_camera-rs0
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf  ~/calib_camera-rs1 \
#   && cd $COLCON_WS \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_camera_record 1 /home/chutsu/calib_camera-rs1
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf  ~/calib_camimu-rs0 \
#   && cd $COLCON_WS \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_camimu_record 0 /home/chutsu/calib_camimu-rs0
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf  ~/calib_camimu-rs1 \
#   && cd $COLCON_WS \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_camimu_record 0 /home/chutsu/calib_camimu-rs1
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf  ~/calib_gimbal \
#   && cd $COLCON_WS \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_gimbal_record2 /home/chutsu/calib_gimbal
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   rm -rf ~/calib_gimbal_data \
#   && cd $COLCON_WS \
#   && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 calib_gimbal_inspect
# " C-m C-m

tmux send-keys -t dev -R C-l C-m
tmux send-keys -t dev -R "\
  cd $COLCON_WS \
  && colcon build --packages-select proto_ros2 \
  && source install/setup.bash \
  && ros2 run proto_ros2 calib_gimbal_inspect2 /data/gimbal_experiments/calib/calib_gimbal/calib_gimbal-results.yaml
" C-m C-m

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

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd $COLCON_WS \
#   && colcon build --packages-select proto_ros2 \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 rs_node
# " C-m C-m

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
# cd $COLCON_WS \
#   && colcon build --packages-select proto_ros2 \
#   && source install/setup.bash \
#   && ros2 run proto_ros2 rs_multicam_node
# " C-m C-m

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
