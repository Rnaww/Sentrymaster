#!/bin/bash
cd ..
source install/setup.bash
ros2 launch sentry_sim simulator_701.launch.py physics:=gazebo gui:=true rviz:=true "$@"
