#!/bin/bash
set -e
COLCON_WS=~/colcon_ws
source $COLCON_WS/install/setup.bash

# python3 scripts/okvis_calib_converter.py
# python3 scripts/inspect_data.py
# python3 scripts/calib_gimbal.py
python3 scripts/experiment.py
# python3 scripts/inspect_data.py
# python3 src/mav_node.py

# CMD="rm -rf $HOME/calib_camera-rs0 && ros2 run proto_ros2 calib_camera_record 0 $HOME/calib_camera-rs0"
# CMD="rm -rf $HOME/calib_camera-rs1 && ros2 run proto_ros2 calib_camera_record 1 $HOME/calib_camera-rs1"
# CMD="rm -rf $HOME/calib_camimu-rs0 && ros2 run proto_ros2 calib_camimu_record 0 $HOME/calib_camimu-rs0"
# CMD="rm -rf $HOME/calib_camimu-rs1 && ros2 run proto_ros2 calib_camimu_record 0 $HOME/calib_camimu-rs1"
# CMD="rm -rf $HOME/calib_gimbal && ros2 run proto_ros2 calib_gimbal_record2 $HOME/calib_gimbal"
# CMD="ros2 run proto_ros2 calib_gimbal_inspect2 $HOME/calib-240212/calib_gimbal/calib_gimbal-results.yaml"
# CMD="ros2 launch proto_ros2 gimbal_experiments.launch.py"
# CMD="ros2 launch proto_ros2 vicon_experiments.launch.py"

# tmux send-keys -t dev -R C-l C-m
# tmux send-keys -t dev -R "\
#   cd $COLCON_WS \
#   && colcon build --packages-select proto_ros2 \
#   && colcon build --packages-select ros2_vicon \
#   && colcon build --packages-select okvis \
#     --cmake-args -DCMAKE_BUILD_TYPE=Release \
#     -DUSE_NN=OFF \
#     -DBUILD_SUPEREIGHT2_APP=OFF \
#     -DBUILD_ROS2=ON \
#   && source install/setup.bash \
#   && $CMD
# " C-m C-m
