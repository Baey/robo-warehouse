import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Correct import for Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robo_warehouse = get_package_share_directory('robo_warehouse')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f"-r {os.path.join(pkg_robo_warehouse, 'worlds', 'tugbot_depot.sdf')}"}.items(),
    )

    ros2gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', 'config_file:=/home/developer/ros2_ws/src/robo_warehouse/config/ros2gz_bridge_config.yaml'],
        output='screen'
    )
<<<<<<< HEAD

    return LaunchDescription([
        gz_sim,
        ros2gz_bridge
=======
    

    return LaunchDescription([
        gz_sim,
        ros2gz_bridge,
        tugbot_controller,
        DeclareLaunchArgument(
            'urdf_file',
            default_value='/home/developer/ros2_ws/src/robo_warehouse/models/Tugbot/model.urdf'
,
            description='Full path to the URDF file to load'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[LaunchConfiguration('urdf_file')],
            output='screen'
        )
>>>>>>> 495caef (slam ready)
    ])
