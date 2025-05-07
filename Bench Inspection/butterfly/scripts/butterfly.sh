#!/bin/bash

# Terminal 1 - Runs butterfly workspace  build setup, and launches the drone simulation in Gazebo Harmonic.
gnome-terminal -- bash -c "cd ~/butterfly && \
colcon build && \
source ~/butterfly/install/setup.bash && \
ros2 launch butterfly launch.py &&\
echo 'butterfly simulation set up successful!' && exec bash"