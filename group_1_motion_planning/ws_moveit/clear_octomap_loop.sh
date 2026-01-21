#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/Project-3-1/ws_moveit/devel/setup.bash

echo "Clearing octomap every 2 seconds. Ctrl+C to stop."

while true; do
  rosservice call /clear_octomap
  sleep 2
done
