import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Correct import for Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robo_warehouse = get_package_share_directory('robo_warehouse')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup') 

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f"-r {os.path.join(pkg_robo_warehouse, 'worlds', 'tugbot_depot.sdf')}"}.items(),
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("robo_warehouse"), 'config', 'slam_params.yaml']
            )
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(pkg_robo_warehouse, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    ros2gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f"config_file:={os.path.join(pkg_robo_warehouse, 'config', 'ros2gz_bridge_config.yaml')}"],
        output='screen',
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('robo_warehouse'), 'config', 'robot_navigation.rviz']
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        slam_node,
        ros2gz_bridge,
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
            parameters=[{'robot_description': open('/home/developer/ros2_ws/src/robo_warehouse/models/Tugbot/model.urdf').read()}],
            output='screen'
        ),
        nav2_launch,
        rviz_node
    ])
