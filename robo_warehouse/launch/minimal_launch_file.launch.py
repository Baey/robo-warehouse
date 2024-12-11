import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Correct import for Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robo_warehouse = get_package_share_directory('robo_warehouse')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f"-r {os.path.join(pkg_robo_warehouse, 'worlds', 'tugbot_depot.sdf')}"}.items(),
    )

    # tugbot_controller = Node(
    #     package='robo_warehouse',
    #     executable='tugbot_driver',
    #     name='tugbot_driver',
    #     output='screen'
    # )

    ros2gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', 'config_file:=/home/developer/ros2_ws/src/robo_warehouse/ros2gz_bridge_config.yaml'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        ros2gz_bridge,
        # tugbot_controller
    ])
