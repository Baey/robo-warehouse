import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Correct import for Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robo_warehouse = get_package_share_directory('robo_warehouse')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f"-r {os.path.join(pkg_robo_warehouse, 'worlds', 'tugbot_depot.sdf')}"}.items(),
    )

    tugbot_controller = Node(
        package='robo_warehouse',
        executable='robo_warehouse_entry',
        name='tugbot_controller',
        output='screen'
    )

    ros2gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', 'config_file:=/home/developer/ros2_ws/src/robo_warehouse/ros2gz_bridge_config.yaml'],
        output='screen'
    )
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
    # )

    return LaunchDescription([
        gz_sim,
        ros2gz_bridge,
        tugbot_controller
        # robot_state_publisher
    ])
