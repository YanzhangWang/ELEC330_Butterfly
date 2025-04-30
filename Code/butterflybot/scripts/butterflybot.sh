#!/bin/bash

# Terminal 1 - Runs butterflybot workspace  build setup, and launches the drone simulation in Gazebo Harmonic.
gnome-terminal -- bash -c "cd ~/butterflybot && \
colcon build && \
source ~/butterflybot/install/setup.bash && \
ros2 launch butterflybot launch.py &&\
echo 'butterflybot simulation set up successful!' && exec bash"