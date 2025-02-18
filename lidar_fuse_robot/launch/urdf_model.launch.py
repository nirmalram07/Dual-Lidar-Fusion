import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get URDF/Xacro path
    pkg_path = get_package_share_directory('lidar_fuse_robot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'lidar_bot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Robot State Publisher Node with Xacro Parameters
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch Description
    return LaunchDescription([
         DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
