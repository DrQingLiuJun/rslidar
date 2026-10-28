#!/bin/bash
set -eo pipefail
cd /home/user/carla-projects/rslidar_ws
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=82 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export CUDA_HOME=/usr/local/cuda-12.1
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate
cd vendor/OpenPCDet
exec ros2 launch rslidar_sdk run_3_detectors.launch.py
