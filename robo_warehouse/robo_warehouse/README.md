bridge first
source install/setup.bash
colcon build --symlink-install
ros2 run robo_warehouse robo_warehouse_entry
colcon clean workspace
ros2 launch robo_warehouse minimal_launch_file.launch.py

ros2 run tf2_tools view_frames
ros2 launch slam_toolbox online_async_launch.py
sudo apt install ros-humble-slam-toolbox ros-humble-rtabmap-ros
