#!/bin/bash
set -e

cd /home/developer/ros2_ws

# vcs import --recursive < /home/developer/ros2_ws/src/px4.repos src

rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
echo "export ROS_DOMAIN_ID=21" >> ~./bashrc
