#!/bin/bash

source /opt/ros/jazzy/setup.bash
cd /home/rov_ws
colcon build --symlink-install --parallel-workers 4