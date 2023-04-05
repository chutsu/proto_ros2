#!/bin/bash
set -e

# export GZ_SIM_SERVER_CONFIG_PATH=$PWD/server.config
export GZ_SIM_RESOURCE_PATH=$PWD/worlds:$PWD/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build
# mkdir -p build && cd build && cmake .. && make
# gz sim calibration.sdf --gui-config ~/.gz/sim/7/gui.config
# gz sim sandbox.sdf --gui-config gui.config
